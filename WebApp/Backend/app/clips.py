# Handling video clips 
# Robot caputres clip -> calls this api -> stores clip in db and s3

from fastapi import APIRouter, UploadFile, File, Form
from datetime import datetime
from images import storeUserMedia
from config import connect_db

clips_router = APIRouter()

@clips_router.post("/upload")
def upload_clip(robot_id: str = Form(...), video: UploadFile = File(...)):
    """
     
    """
    conn = connect_db()
    cursor = conn.cursor()

    try:
        cursor.execute("SELECT user_id FROM robots where id=%s", (robot_id),)
        user_id = cursor.fetchone()

        user_id = user_id[0]
        cursor.execute("SELECT cognito_sub FROM users where id =%s",(user_id),)

        user_uuid = cursor.fetchone()

        user_uuid = user_uuid[0]

        storeUserMedia(user_uuid, video)

        return{"message": "Video uploaded successfully", "user_uuid": user_uuid}

    except Exception as e:
        print(f"Failed to uplod video {e}")
    

# for returning presigned url for clip of the robot
@clips_router.get("/{clip_id}/url")
def get_clip_url(clip_id: str):
    pass


