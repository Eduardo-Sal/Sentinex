# notifications endpoints

from fastapi import APIRouter, UploadFile, File, Form, HTTPException
from datetime import datetime
import time
from config import connect_db, s3, s3_bucket
from pydantic import BaseModel
from images import get_presigned_url_v2

# TODO optimize later since all methods use a database connection

notifications_router = APIRouter()

# optimize later with pagination or filtering
# TODO add clips support later
@notifications_router.get("/user/{user_uuid}")
def get_user_notification(user_uuid: str):
    # connect to db
    conn = connect_db()
    cursor = conn.cursor()

    # user user_id as the filter
    try:
        query = "SELECT id FROM users WHERE cognito_sub = %s"
        cursor.execute(query, (user_uuid,))
        data = cursor.fetchall()

        user_id = data[0][0]  # extract id
        print(user_id)

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
                "id": notification_id,
                "s3_filename": s3_filename,
                "file_url": presigned_url,
                "timestamp": timestamp,
                "media_type": media_type
            })

        return notifications
    except Exception as e:
        print(f"str{e}")
        raise HTTPException(status_code=500, detail = "error fetching notifications")
    finally:
        cursor.close()
        conn.close()

# TODO optimize later with dictionaries instead of tuples, just cleaner
# TODO optimize path as well 
@notifications_router.delete("/notification/{notification_id}")
def delete_user_notification(notification_id: int): 
    # connect to db
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # Extract file path of s3 bucket
        query = "SELECT s3_filename FROM notifications WHERE notification_id = %s"
        cursor.execute(query, (notification_id,))
        result = cursor.fetchone()

        if not result:
            raise HTTPException(status_code=404, detail="Notification doesn't exist")
        
        s3_filepath = result[0]

        # Delete image given Key which is the path
        if s3_filepath:
            try:
                s3.delete_object(Bucket=s3_bucket,Key=s3_filepath)
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to delete {str(e)}")
            
        # delete the row and delete the s3 image from s3 bucket
        query = "DELETE FROM notifications WHERE notification_id = %s"
        cursor.execute(query,(notification_id,))
        conn.commit()

    # in terms of error the notification SHOULD exist since the user will manually delete it using the UI
        return {"message": f"Notification {notification_id} deleted successfully"}
    except Exception as e:
        print(f"Error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")
    finally:
        cursor.close()
        conn.close()
        
            
@notifications_router.delete("/users/{user_uuid}/notifications")
def delete_all_user_notifications(user_uuid: str):
    # connect to db
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # search for specific notification using user_id
        query = "select id FROM users WHERE cognito_sub = %s"
        cursor.execute(query,(user_uuid))
        data = cursor.fetchall()

        user_id = data[0][0]
        
        query = "SELECT s3_filename FROM notifications WHERE user_id = %s"
        cursor.execute(query,(user_id,))
        results = cursor.fetchall()

        # delete ALL the s3 images related to user_id using for loop
        for row in results:
            s3_filepath = row[0]
            if s3_filepath:
                try:
                    s3.delete_object(Bucket=s3_bucket, Key= s3_filepath)
                except Exception as e:
                    print(f"{str(e)}")
    
        query = "DELETE FROM notifications where user_id = %s"
        cursor.execute(query,(user_id,))
        conn.commit()

        return {"message": f"All notifications for user {user_id} deleted successfully"}
    
    except Exception as e:
        return print(f"{str(e)}")
    finally:
        cursor.close()
        conn.close()
    
