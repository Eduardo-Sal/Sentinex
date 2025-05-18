// src/components/dashboard/DirectionPad.tsx
import Arrow from "./ArrowCamera";
import "./direction_pad.css";

const DirectionPadCamera = () => {
  return (
    <div className="arrow-container">
      <Arrow mode="left" id="arrow-left" />
      <div id="center" />
      <Arrow mode="right" id="arrow-right" />
    </div>
  );
};

export default DirectionPadCamera;