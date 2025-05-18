import { AmplifyUser } from "@aws-amplify/ui";
import { useEffect } from "react";
import { PubSub } from "aws-amplify";
import  DirectionPad from "./DirectionPad";

export interface ControllerProps {
  user?: AmplifyUser;
}

const Controller = () => {

  return (
    <>
      <DirectionPad />
    </>
  );
};

export default Controller;