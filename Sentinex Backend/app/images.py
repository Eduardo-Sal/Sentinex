# same as clips.py except with screenshots
# handles screenshot sent from robot & metadata

from fastapi import APIRouter, UploadFile,File, Form, HTTPException
from datetime import datetime
from config import connect_db, s3_bucket, s3
from pydantic import BaseModel

images_router = APIRouter()

class imageGet(BaseModel):
    user_id: str
    filename: str

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
            folder = "screenshot"
        elif content_type.startswith("video/"):
            folder = "clips"
        else:
            print(f"Unsupported file type: {content_type}")
            return


        filePath = f"{user_id}/{folder}/{media.filename}"
        s3.put_object(
            Bucket = s3_bucket,
            Key = filePath,
            Body = media_decoded,
            ContentType = content_type
        )
        print(f"Stored Image at s3://{s3_bucket}/{filePath}")
    except Exception as e:
        print(f"Failed to store media {e}")



@images_router.get("/media")
def get_presigned_url(request: imageGet):
    conn = connect_db()
    cursor = conn.cursor()
    try:
        cursor.execute("SELECT cognito_sub FROM users WHERE id = %s",(request.user_id,))
        user_uuid = cursor.fetchone()
        user_uuid = user_uuid[0]

        image_path = f"{user_uuid}/screenshot/{request.filename}"
        video_path = f"{user_uuid}/clips/{request.filename}"

        for file_path in [image_path, video_path]:
            try:
                s3.head_object(Bucket=s3_bucket, Key=file_path)
                presigned_url = s3.generate_presigned_url(
                    "get_object",
                    Params = {"Bucket": s3_bucket, "Key": file_path},
                    ExpiresIn=600
                )
                return {"url": presigned_url}
            except s3.exceptions.ClientError:
                continue

        raise HTTPException(status_code=404,detail="aint here")
    except Exception as e:
        print(f"{e}")

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
    


# obtain all the alerts related to the robot_id paired with the user so Users table will have robot_id or try with user_id
@images_router.get("/get-alerts/{user_id}")
def getAllalerts(screenshot_id: str):
    '''Gets all alerts from the Database associated with the user_id '''
