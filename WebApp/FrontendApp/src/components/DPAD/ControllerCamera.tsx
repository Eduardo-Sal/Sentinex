import { AmplifyUser } from "@aws-amplify/ui";
import { useEffect } from "react";
import { PubSub } from "aws-amplify";
import  DirectionPadCamera from "./DirectionPadCamera";

export interface ControllerProps {
  user?: AmplifyUser;
}

const ControllerCamera = () => {
  return (
    <>
      <DirectionPadCamera />
    </>
  );
};

export default ControllerCamera;