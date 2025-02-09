# same as clips.py except with screenshots
# handles screenshot sent from robot & metadata

from fastapi import APIRouter
import boto3
import uuid
from datetime import datetime

router = APIRouter()
s3 = boto3.client("") # add later but s3
dynamodb = boto3.resource("") # add later but it's dynamo
screenshots_table = dynamodb.Table("") # add later table schema wil lbe "Screenshots"
S3_BUCKET = "" # add later need to configure it in AWS

# uploads screenshot to s3 and metadata in dynamo
@router.post("/{robot_id}/upload")
def upload_screenshot(robot_id: str, data:dict):
    pass

# for returning presigned url for screenshot of the robot
@router.get("/{screenshot_id}/url")
def get_presigned_url(screenshot_id: str):
    pass

