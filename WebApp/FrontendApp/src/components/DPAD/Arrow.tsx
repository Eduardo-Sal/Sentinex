// src/components/dashboard/Arrow.tsx
import React, { useCallback } from "react";
import { PubSub } from "@aws-amplify/pubsub";
import { debounce } from "../../utils/Debounce";
import "./direction_pad.css";

interface ArrowProps {
  command: "up" | "down" | "left" | "right";
  id: string;
}

const DEBOUNCE_DELAY = 600; // adjust delay as needed

const Arrow = ({ command, id }: ArrowProps) => {
  // Create callback functions for publish actions.
  const publishPress = useCallback(() => {
    console.log(`Click on ${command} arrow`);
    PubSub.publish("robot-4/direction", {
      command,
    }).catch(console.error);
  }, [command]);

  /*const publishRelease = useCallback(() => {
    console.log(`Release on ${direction} arrow`);
    PubSub.publish("Sentinex/pub", {
      direction,
      enabled: false,
      timestamp: new Date().toISOString(),
    }).catch(console.error);
  }, [direction]);*/

  // Wrap the publish calls in debounced functions.
  const debouncedPublishPress = useCallback(debounce(publishPress, DEBOUNCE_DELAY), [publishPress]);
 // const debouncedPublishRelease = useCallback(debounce(publishRelease, DEBOUNCE_DELAY), [publishRelease]);

  // Event handlers call the debounced functions.
  const onButtonPress = () => {
    debouncedPublishPress();
  };

  /*const onButtonRelease = () => {
    debouncedPublishRelease();
  };*/

  // Prevent context menu on long press/right-click
  const disableContext = (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    return false;
  };

  return (
    <div
      id={id}
      className={`arrow-button ${command}`}
      onPointerDown={onButtonPress}
     // onPointerUp={onButtonRelease}
      onContextMenu={disableContext}
    />
  );
};

export default Arrow;