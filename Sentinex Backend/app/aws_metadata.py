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
        # Fetch AWS metadata
        cursor.execute("""
            SELECT id, aws_metadata FROM robots WHERE user_id = (
                SELECT id FROM users WHERE cognito_sub = %s
            ) LIMIT 1;
        """, (user_uuid,))
        robot_row = cursor.fetchone()

        if not robot_row:
            raise HTTPException(status_code=404, detail="No robot found for this user.")

        robot_id, aws_metadata_json = robot_row
        aws_metadata = json.loads(aws_metadata_json)

        # Fetch recent 5 notifications including db_level
        cursor.execute("""
            SELECT event_type, timestamp, db_level FROM notifications
            WHERE user_id = (SELECT id FROM users WHERE cognito_sub = %s)
            ORDER BY timestamp DESC
            LIMIT 5;
        """, (user_uuid,))
        notifications = cursor.fetchall()
        recent_events = [
            {
                "event_type": row[0],
                "timestamp": row[1].isoformat() if hasattr(row[1], 'isoformat') else str(row[1]),
                "db_level": row[2]
            }
            for row in notifications
        ]

        aws_metadata["recent_events"] = recent_events

        return aws_metadata

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to fetch AWS info: {str(e)}")

    finally:
        cursor.close()
        conn.close()