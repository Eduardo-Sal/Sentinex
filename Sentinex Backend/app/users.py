# users.py

from fastapi import APIRouter, HTTPException, Query
from pydantic import constr
from config import connect_db

users_router = APIRouter()

@users_router.get("/robot-id")
def get_robot_id(user_uuid: constr(min_length=36, max_length=36) = Query(...)):
    """
    Fetch the robot_id associated with a user (by Cognito UUID)
    """
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # Get user_id from Cognito UUID
        cursor.execute("SELECT id FROM users WHERE cognito_sub = %s;", (user_uuid,))
        user = cursor.fetchone()
        if not user:
            raise HTTPException(status_code=404, detail="User not found.")
        user_id = user[0]

        # Get robot_id from robots table
        cursor.execute("SELECT id FROM robots WHERE user_id = %s;", (user_id,))
        robot = cursor.fetchone()
        if not robot:
            raise HTTPException(status_code=404, detail="No robot paired with this user.")

        robot_id = robot[0]
        return {"robot_id": robot_id}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to fetch robot: {str(e)}")

    finally:
        cursor.close()
        conn.close()