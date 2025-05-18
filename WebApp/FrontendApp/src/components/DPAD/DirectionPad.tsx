// src/components/dashboard/DirectionPad.tsx
import Arrow from "./Arrow";
import "./direction_pad.css";

const DirectionPad = () => {

  return (
    <div className="arrow-container">
      <div />
      <Arrow command="up" id="arrow-up" />
      <div />
      <Arrow command="left" id="arrow-left" />
      <div id="center" />
      <Arrow command="right" id="arrow-right" />
      <div />
      <Arrow command="down" id="arrow-down" />
      <div />
    </div>
  );
};

export default DirectionPad;