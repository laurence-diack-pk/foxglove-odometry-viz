import { ExtensionContext, Immutable, MessageEvent } from "@foxglove/extension";

const DEFAULT_MARKER_OPACITY = 0.8;
const TARGET_BUFFER_SECONDS = 600;
const THINNING_SECONDS = 0.2;
const THINNING_DISTANCE_METERS = 0.2;
const MAX_TRAJECTORY_POINTS = Math.max(
  1,
  Math.ceil(TARGET_BUFFER_SECONDS / Math.max(THINNING_SECONDS, Number.EPSILON)),
);

type Point = { x: number; y: number; z: number };
type Quaternion = { x: number; y: number; z: number; w: number };
type Header = { frame_id?: string; stamp?: unknown };
type OdometryMessage = {
  header: Header;
  pose: { pose: { position: Point; orientation: Quaternion }; covariance?: number[] };
};
type Marker = {
  header: Header;
  ns: string;
  id: number;
  type: number;
  action: number;
  pose: { position: Point; orientation: Quaternion };
  scale: Point;
  color: { r: number; g: number; b: number; a: number };
  points?: Point[];
};
type MarkerArray = { markers: Marker[] };

type TopicState = {
  points: Point[];
  colorIndex: number;
  latest: OdometryMessage;
  lastAcceptedPoint?: Point;
  lastAcceptedTimeSec?: number;
};

function isOdometryMessage(message: unknown): message is OdometryMessage {
  if (message == undefined || typeof message !== "object") {
    return false;
  }

  const record = message as Record<string, unknown>;
  const pose = record["pose"];
  if (pose == undefined || typeof pose !== "object") {
    return false;
  }

  const poseRecord = pose as Record<string, unknown>;
  const nestedPose = poseRecord["pose"];
  if (nestedPose == undefined || typeof nestedPose !== "object") {
    return false;
  }

  const nestedPoseRecord = nestedPose as Record<string, unknown>;
  return nestedPoseRecord["position"] != undefined && nestedPoseRecord["orientation"] != undefined;
}

function readCovarianceDiagonal(covariance: unknown): { x: number; y: number; z: number } {
  if (!Array.isArray(covariance)) {
    return { x: 0, y: 0, z: 0 };
  }

  const x = typeof covariance[0] === "number" ? covariance[0] : 0;
  const y = typeof covariance[7] === "number" ? covariance[7] : 0;
  const z = typeof covariance[14] === "number" ? covariance[14] : 0;
  return {
    x: Math.max(0, x),
    y: Math.max(0, y),
    z: Math.max(0, z),
  };
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function headerStampToSeconds(stamp: unknown): number | undefined {
  if (stamp == undefined || typeof stamp !== "object") {
    return undefined;
  }

  const stampRecord = stamp as Record<string, unknown>;
  const secValue = stampRecord["sec"] ?? stampRecord["secs"];
  const nsecValue = stampRecord["nanosec"] ?? stampRecord["nsec"] ?? stampRecord["nsecs"];

  if (typeof secValue !== "number") {
    return undefined;
  }

  const normalizedNanoseconds = typeof nsecValue === "number" ? nsecValue : 0;
  return secValue + normalizedNanoseconds * 1e-9;
}

function distanceMeters(a: Point, b: Point): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function shouldAcceptTrajectoryPoint(
  position: Point,
  timestampSec: number | undefined,
  state: TopicState,
): boolean {
  if (
    timestampSec == undefined ||
    state.lastAcceptedTimeSec == undefined ||
    state.lastAcceptedPoint == undefined
  ) {
    return true;
  }

  return (
    distanceMeters(position, state.lastAcceptedPoint) >= THINNING_DISTANCE_METERS &&
    timestampSec - state.lastAcceptedTimeSec >= THINNING_SECONDS
  );
}

function isZeroQuaternion(orientation: Quaternion): boolean {
  return (
    orientation.x === 0 &&
    orientation.y === 0 &&
    orientation.z === 0 &&
    (orientation.w === 0 || orientation.w === 1)
  );
}

function hsvToRgb(
  hue: number,
  saturation: number,
  value: number,
): { r: number; g: number; b: number } {
  const chroma = value * saturation;
  const hPrime = hue / 60;
  const x = chroma * (1 - Math.abs((hPrime % 2) - 1));

  let red = 0;
  let green = 0;
  let blue = 0;

  if (hPrime >= 0 && hPrime < 1) {
    red = chroma;
    green = x;
  } else if (hPrime < 2) {
    red = x;
    green = chroma;
  } else if (hPrime < 3) {
    green = chroma;
    blue = x;
  } else if (hPrime < 4) {
    green = x;
    blue = chroma;
  } else if (hPrime < 5) {
    red = x;
    blue = chroma;
  } else {
    red = chroma;
    blue = x;
  }

  const m = value - chroma;
  return {
    r: Number((red + m).toFixed(3)),
    g: Number((green + m).toFixed(3)),
    b: Number((blue + m).toFixed(3)),
  };
}

function colorForTopic(
  index: number,
  opacity: number,
): { r: number; g: number; b: number; a: number } {
  const hue = (index * 137.508) % 360;
  const rgb = hsvToRgb(hue, 0.8, 0.95);
  return { ...rgb, a: clamp(opacity, 0, 1) };
}

function topicIdBase(topicName: string): number {
  let hash = 2166136261;
  for (let index = 0; index < topicName.length; index++) {
    hash ^= topicName.charCodeAt(index);
    hash = Math.imul(hash, 16777619);
  }
  return Math.abs(hash) % 1000000000;
}

function buildMarkersForTopic(topicName: string, state: TopicState): Marker[] {
  const color = colorForTopic(state.colorIndex, DEFAULT_MARKER_OPACITY);
  const idBase = topicIdBase(topicName) * 3;
  const hasValidOrientation = !isZeroQuaternion(state.latest.pose.pose.orientation);
  const covariance = readCovarianceDiagonal(state.latest.pose.covariance);

  const covarianceX = covariance.x;
  const covarianceY = covariance.y;
  const covarianceZ = covariance.z;
  const covarianceScale = {
    x: Math.max(0.001, Math.sqrt(covarianceX) * 2),
    y: Math.max(0.001, Math.sqrt(covarianceY) * 2),
    z: Math.max(0.001, Math.sqrt(covarianceZ) * 2),
  };

  const covarianceMarker: Marker = {
    header: state.latest.header,
    ns: "covariance",
    id: idBase + 1,
    type: 2,
    action: 0,
    pose: {
      position: state.latest.pose.pose.position,
      orientation: hasValidOrientation
        ? state.latest.pose.pose.orientation
        : { x: 0, y: 0, z: 0, w: 1 },
    },
    scale: covarianceScale,
    color: { ...color, a: 0.5 },
  };

  const markers: Marker[] = [];

  if (state.points.length > 0) {
    markers.push({
      header: state.latest.header,
      ns: "trajectory",
      id: idBase,
      type: 4,
      action: 0,
      pose: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
      scale: { x: 0.05, y: 0, z: 0 },
      color,
      points: state.points,
    });
  }

  markers.push(covarianceMarker);

  const currentPoseMarker: Marker = {
    header: state.latest.header,
    ns: "current_pose",
    id: idBase + 2,
    type: hasValidOrientation ? 0 : 2,
    action: 0,
    pose: {
      position: state.latest.pose.pose.position,
      orientation: hasValidOrientation
        ? state.latest.pose.pose.orientation
        : { x: 0, y: 0, z: 0, w: 1 },
    },
    scale: hasValidOrientation ? { x: 0.5, y: 0.1, z: 0.1 } : { x: 0.15, y: 0.15, z: 0.15 },
    color,
  };
  markers.push(currentPoseMarker);

  return markers;
}

function createOdometryToMarkerArrayConverter(): (
  msg: unknown,
  event: Immutable<MessageEvent>,
) => MarkerArray | undefined {
  const statesByTopic = new Map<string, TopicState>();

  return (msg: unknown, event: Immutable<MessageEvent>): MarkerArray | undefined => {
    if (!isOdometryMessage(msg)) {
      return undefined;
    }

    const topicName = event.topic;
    const state = statesByTopic.get(topicName) ?? {
      points: [],
      colorIndex: statesByTopic.size,
      latest: msg,
    };
    const timestampSec = headerStampToSeconds(msg.header.stamp);

    const position = msg.pose.pose.position;
    const shouldResetForTimeJump =
      timestampSec != undefined &&
      state.lastAcceptedTimeSec != undefined &&
      timestampSec < state.lastAcceptedTimeSec;
    if (shouldResetForTimeJump) {
      state.points = [];
      state.lastAcceptedPoint = undefined;
      state.lastAcceptedTimeSec = undefined;
    }

    if (timestampSec != undefined && shouldAcceptTrajectoryPoint(position, timestampSec, state)) {
      state.points.push({ x: position.x, y: position.y, z: position.z });
      if (state.points.length > MAX_TRAJECTORY_POINTS) {
        state.points.splice(0, state.points.length - MAX_TRAJECTORY_POINTS);
      }
      state.lastAcceptedPoint = { x: position.x, y: position.y, z: position.z };
      state.lastAcceptedTimeSec = timestampSec;
    }

    state.latest = msg;
    statesByTopic.set(topicName, state);

    return { markers: buildMarkersForTopic(topicName, state) };
  };
}

export function registerOdometryMarkerConverters(extensionContext: ExtensionContext): void {
  const converter = createOdometryToMarkerArrayConverter();

  for (const fromSchemaName of ["nav_msgs/Odometry", "nav_msgs/msg/Odometry"]) {
    for (const toSchemaName of [
      "visualization_msgs/MarkerArray",
      "visualization_msgs/msg/MarkerArray",
    ]) {
      extensionContext.registerMessageConverter({
        type: "schema",
        fromSchemaName,
        toSchemaName,
        converter,
      });
    }
  }
}
