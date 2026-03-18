import { ExtensionContext } from "@foxglove/extension";

import { registerOdometryMarkerConverters } from "./OrientationViz";

export function activate(extensionContext: ExtensionContext): void {
  registerOdometryMarkerConverters(extensionContext);
}
