# WebRTC/Kinesis video streaming

# look into fastapi websocket
from fastapi import APIRouter, HTTPException
import boto3
from config import aws_region, kinesisvideo, connect_db
import traceback
import json


aws_router = APIRouter()


"""
Fetches AWS metadata (MQTT topics and channel ARN) for the robot linked to the given Cognito user UUID.
"""
@aws_router.get("/awsInfo/{user_uuid}")
def get_aws_info(user_uuid: str):
    conn = connect_db()
    cursor = conn.cursor()
    try:
        cursor.execute("""
            SELECT aws_metadata FROM robots WHERE user_id = (
                SELECT id FROM users WHERE cognito_sub = %s
            ) LIMIT 1;
        """, (user_uuid,))
        robot_row = cursor.fetchone()

        if not robot_row:
            raise HTTPException(status_code=404, detail="No robot found for this user.")

        # Parse the JSON string to dict
        aws_metadata = json.loads(robot_row[0])

        return aws_metadata

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to fetch AWS info: {str(e)}")
    
    finally:
        cursor.close()
        conn.close()