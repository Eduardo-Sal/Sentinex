# notifications endpoints

from fastapi import APIRouter, UploadFile, File, Form, HTTPException
from datetime import datetime
import time
from config import connect_db
from pydantic import BaseModel
from images import get_presigned_url_v2

# TODO optimize later since all methods use a database connection

notifications_router = APIRouter()

# optimize later with pagination or filtering
# TODO add clips support later
@notifications_router.get("/user/{user_id}")
def get_user_notification(user_id: int):
    # connect to db
    conn = connect_db()
    cursor = conn.cursor()

    # user user_id as the filter
    try:
        query = """
            SELECT notification_id, s3_filename, timestamp, media_type 
            FROM notifications 
            WHERE user_id = %s
        """
        cursor.execute(query,(user_id,))
        rows = cursor.fetchall()
        print(f"Fetched {len(rows)} rows")  # Debug log

        if not rows:
            print("User doesn't exist")
            return None
        
        notifications = []


        for row in rows:
            notification_id, s3_filename, timestamp, media_type = row
            presigned_url = None
        # return all the notifications along with file path 
            if s3_filename:
                presigned_url = get_presigned_url_v2(s3_filename)
            print(f"{notification_id}")
            notifications.append({
                "id:": notification_id,
                "s3_filename": s3_filename,
                "file_url": presigned_url,
                "timestamp": timestamp,
                "media_type": media_type
            })

        return notifications
    except Exception as e:
        raise HTTPException(status_code=500, detail = "error fetching notifications")
    finally:
        cursor.close()
        conn.close()


#@notifications_router.delete()
def delete_user_notification():
    pass
    
#@notifications_router.delete()
def delete_all_user_notifications():
    pass
