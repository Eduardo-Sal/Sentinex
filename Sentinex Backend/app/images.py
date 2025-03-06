# same as clips.py except with screenshots
# handles screenshot sent from robot & metadata

from fastapi import APIRouter, UploadFile,File, Form
import boto3
import uuid
from datetime import datetime
from config import cognito, connect_db, app_client_id, s3_bucket, s3, aws_region, user_pool_id

images_router = APIRouter()

def storeUserMedia(user_id:str, media):
    """
    Stores image into S3 bucket associated with User_ID
    """
    try:
        # decode media bytes
        media_decoded = media.file.read()
        # determine content_type for s3 upload
        content_type = media.content_type

        # Check if it's an image or video
        if content_type.startswith("image/"):
            folder = "screenshots"
        elif content_type.startswith("video/"):
            folder = "clips"
        else:
            print(f"Unsupported file type: {content_type}")
            return


        filePath = f"{user_id}/screenshot/{media.filename}"
        s3.put_object(
            Bucket = s3_bucket,
            Key = filePath,
            Body = media_decoded,
            ContentType = content_type
        )
        print(f"Stored Image at s3://{s3_bucket}/{filePath}")
    except Exception as e:
        print(f"Failed to store image {e}")


@images_router.post("/upload")
def upload_screenshot(robot_id: str  = Form(...), image: UploadFile = File(...)):
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # get user_id
        cursor.execute("SELECT user_id FROM robots where id=%s",(robot_id,))
        user_id = cursor.fetchone()
        
        #handle error
        if not user_id:
            print("No user ID associated with the robot_id, but that's impossible how did the robot do this request without a user?!?!")
            return None

        # get val from tuple
        user_id = user_id[0]

        # get user_uuid
        cursor.execute("SELECT cognito_sub FROM users where id=%s",(user_id,))
        user_uuid = cursor.fetchone()

        # handle error
        if not user_uuid:
            print("Doesn't exist")
            return None
        user_uuid = user_uuid[0]

        storeUserMedia(user_uuid, image)

        return {"message": "Image uploaded successfully", "user_uuid": user_uuid}

    except Exception as e:
        print(f"Failed to upload image {e}")
    

# for returning presigned url for screenshot of the robot for viewing 
@images_router.get("/{screenshot_id}/url")
def get_presigned_url(screenshot_id: str):
    pass

# obtain all the alerts related to the robot_id paired with the user so Users table will have robot_id or try with user_id
@images_router.get("/get-alerts/{user_id}")
def getAllalerts(screenshot_id: str):
    '''Gets all alerts from the Database associated with the user_id '''

    
