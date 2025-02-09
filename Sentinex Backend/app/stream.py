# WebRTC/Kinesis video streaming

# look into fastapi websocket
from fastapi import APIRouter, HTTPException
import boto3
from config import AWS_REGION

router = APIRouter()
kinesis_client = boto3.client("kinesis_video_stream", region_name=AWS_REGION)


# starts the stream for the robot
@router.post("/{robot_id}/start_stream")
def start_stream(robot_id: str, stream_type: str):
    pass 

# stops the stream for the robot
@router.post("/{robot_id}/stop_stream")
def stop_stream(robot_id: str):
    pass