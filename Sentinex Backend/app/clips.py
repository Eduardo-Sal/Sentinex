# Handling video clips 
# Robot caputres clip -> calls this api -> stores clip in db and s3

from fastapi import APIRouter
import boto3
import uuid
from datetime import datetime

router = APIRouter()
s3 = boto3.client("")
dynamodb = boto3.resource("") 
clips_table = dynamodb.Table("") # add later table schema wil lbe "Clips"
S3_BUCKET = "" 

# uploads clip to s3 and metadata in dynamo
@router.post("/{robot_id}/upload")
def upload_clip(robot_id: str, data:dict):
    pass
# for returning presigned url for clip of the robot
@router.get("/{clip_id}/url")
def get_clip_url(clip_id: str):
    pass