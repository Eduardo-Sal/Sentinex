# same as clips.py except with screenshots
# handles screenshot sent from robot & metadata

from fastapi import APIRouter, UploadFile,File, Form, HTTPException
import uuid
from datetime import datetime
from config import connect_db, s3_bucket, s3
from pydantic import BaseModel

images_router = APIRouter()

class imageGet(BaseModel):
    user_id: str
    filename: str

def storeUserMedia(user_id: str, media, folder_override: str = None):
    """
    Stores image or video into S3 bucket associated with user_id.
    Optionally override the folder (e.g., known_faces).
    """
    try:
        # decode media bytes
        media_decoded = media.file.read()
        # determine content_type for s3 upload
        content_type = media.content_type

        # Set folder automatically or use override
        if folder_override:
            folder = folder_override
            mediaType = "image"  # assuming known faces are always images
        else:
            if content_type.startswith("image/"):
                folder = "screenshot"
                mediaType = "image"
            elif content_type.startswith("video/"):
                folder = "clips"
                mediaType = "clip"
            else:
                print(f"Unsupported file type: {content_type}")
                return

        filePath = f"{user_id}/{folder}/{media.filename}"
        s3.put_object(
            Bucket=s3_bucket,
            Key=filePath,
            Body=media_decoded,
            ContentType=content_type
        )
        print(f"Stored media at s3://{s3_bucket}/{filePath}")
        return (filePath, mediaType)

    except Exception as e:
        print(f"Failed to store media {e}")
        return None, None

# TODO implement LRU caching to avoid duplicate calls for generating the same
# pre-signed url
def get_presigned_url_v2(file_path: str):
    try:
        url = s3.generate_presigned_url(
            ClientMethod = 'get_object',
            Params = {'Bucket': s3_bucket, 'Key': file_path},
            ExpiresIn = 600
        )
        return url
    except Exception as e:
        raise HTTPException(status_code=500, detail="Error generating pre-signed URL")

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

        raise HTTPException(status_code=500,detail="aint here")
    except Exception as e:
        print(f"{e}")
    finally:
        cursor.close()
        conn.close()

# TODO think about confident score do we also store it?
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

        s3_filepath, mediatype = storeUserMedia(user_uuid, image)
        print(s3_filepath)
        print(mediatype)
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

        if not s3_filepath or not mediatype:
            print("S3 upload failed. Skipping database insert.")
            return {"error": "Failed to upload media to S3"}

        cursor.execute(
            """
            INSERT INTO notifications (user_id, robot_id, s3_filename, timestamp, media_type, event_type)
            VALUES (%s, %s, %s, %s, %s, %s)
            """,
            (user_id, robot_id, s3_filepath, timestamp, mediatype, 'human_detected')
        )
        conn.commit()

        return {"message": "Image uploaded to S3 bucket and Notification Table successfully", "user_uuid": user_uuid}

    except Exception as e:
        print(f"Failed to upload image {e}")
    finally:
        conn.close()
        cursor.close()
    


# obtain all the alerts related to the robot_id paired with the user so Users table will have robot_id or try with user_id
@images_router.get("/get-alerts/{user_id}")
def getAllalerts(screenshot_id: str):
    '''Gets all alerts from the Database associated with the user_id '''

@images_router.post("/register-face")
def register_face(name: str = Form(...), robot_id: str = Form(...), user_uuid: str = Form(...), image: UploadFile = File(...)):
    """
    Registers a new face image by uploading it to S3 under the known_faces folder
    and storing a record in the known_faces database table.
    """
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # Save to S3 under known_faces folder
        s3_filepath, _ = storeUserMedia(user_uuid, image, folder_override='known_faces')
        if not s3_filepath:
            return {"error": "Failed to upload face to S3"}

        # Insert record into known_faces table
        cursor.execute(
            """
            INSERT INTO known_faces (robot_id, name, s3_filename)
            VALUES (%s, %s, %s)
            """,
            (robot_id, name, s3_filepath)
        )
        conn.commit()

        return {"message": "Face registered successfully", "s3_path": s3_filepath}

    except Exception as e:
        print(f"Failed to register face {e}")
        return {"error": str(e)}

    finally:
        cursor.close()
        conn.close()

@images_router.post("/notifications/face")
def face_notification(name: str = Form(...), robot_id: str = Form(...), image: UploadFile = File(...)):
    """
    Uploads a detected face image to S3 and logs a notification.
    """
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # Get user_id from robot_id
        cursor.execute("SELECT user_id FROM robots WHERE id=%s", (robot_id,))
        user_id = cursor.fetchone()
        if not user_id:
            return {"error": "Robot not linked to user"}
        user_id = user_id[0]

        # Get user_uuid
        cursor.execute("SELECT cognito_sub FROM users WHERE id=%s", (user_id,))
        user_uuid = cursor.fetchone()
        if not user_uuid:
            return {"error": "User UUID not found"}
        user_uuid = user_uuid[0]

        # Upload image to S3 under screenshot/
        s3_filepath, _ = storeUserMedia(user_uuid, image, folder_override='screenshot')
        if not s3_filepath:
            return {"error": "Failed to upload face image to S3"}

        # Determine event_type and media_type
        if name.lower() == 'unknown':
            event_type = 'face'
            media_type = 'face'
        else:
            event_type = name  # store the actual name in event_type
            media_type = 'known_face'
        # Insert notification row
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        cursor.execute(
            """
            INSERT INTO notifications (user_id, robot_id, s3_filename, timestamp, media_type, event_type)
            VALUES (%s, %s, %s, %s, %s, %s)
            """,
            (user_id, robot_id, s3_filepath, timestamp, media_type, event_type)
        )
        conn.commit()
        return {"message": f"Face notification created for {name}", "s3_path": s3_filepath}

    except Exception as e:
        print(f"Failed to create face notification {e}")
        return {"error": str(e)}
    finally:
        conn.close()
        cursor.close()