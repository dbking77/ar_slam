import { ExtensionContext } from "@foxglove/studio";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerMessageConverter({
    fromSchemaName: "ar_slam_interfaces/msg/Capture",
    toSchemaName: "sensor_msgs/msg/Image",
    //converter: (captureMsg: Capture, messageEvent: MessageEvent<Capture>) => {
    converter: (captureMsg: any) => {
      return captureMsg.image;
    },
  });
}
