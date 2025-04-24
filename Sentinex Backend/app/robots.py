# Robot registration & functionality like controlling it and streams
# Optimize later by including HTTPException and depends

from fastapi import APIRouter, HTTPException
from dotenv import load_dotenv
from pydantic import BaseModel
from config import cognito, connect_db, app_client_id, s3_bucket, s3, aws_region, user_pool_id, kinesisvideo
from typing import Optional
from datetime import datetime, timedelta

class DevicePair(BaseModel):
    user_uuid: str
    robot_id: str

class RobotRegistration(BaseModel):
    robot_uuid: str

class PairingRequest(BaseModel):
    pairing_enabled: bool
    pairing_duration_minutes: Optional[int] = 3 # defaults to 3 minutes if not specified

robots_router = APIRouter()

@robots_router.patch("/{robot_id}/discovery")
def update_pairing(robot_id: int, payload: PairingRequest):
    '''
    Enables discovery mode for pairing on robot sets 
    '''
    try:
        conn = connect_db()
        cursor = conn.cursor()

        if payload.pairing_enabled:
            expires_at = datetime.utcnow() + timedelta(minutes = payload.pairing_duration_minutes)
            cursor.execute(
                """
                UPDATE robots
                SET pairing_enabled = %s,
                    pairing_expires_at = %s
                WHERE id = %s;
                """,
                (1, expires_at, robot_id)
            )
        else:
            cursor.execute(
            """
            UPDATE robots
            SET pairing_enabled = %s,
                pairing_expires_at = NULL
            WHERE id = %s;
            """,
            (0, robot_id)
            )
        conn.commit()
        return {
            "success": True,
            "robot_id": robot_id,
            "pairing_enabled:" : payload.pairing_enabled
            }

    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to enable discovery mode {str(e)}")

    finally:
        if cursor:
            cursor.close()
        if conn:
            conn.close()
@robots_router.post("/register")
def register_robot(data: RobotRegistration):
    '''
    Registers robot and creates a Kinesis signaling channel per robot
    '''
    conn = connect_db()
    cursor = conn.cursor()
    try:
        cursor.execute("SELECT id, channel_arn FROM robots WHERE uuid = %s;", (data.robot_uuid,))
        existing_robot = cursor.fetchone()

        if existing_robot:
            return {"robot_id": existing_robot[0], "channel_arn": existing_robot[1]}

        # Create Kinesis signaling channel
        channel_name = f"robot-{data.robot_uuid}"
        response = kinesisvideo.create_signaling_channel(
            ChannelName=channel_name,
            ChannelType="SINGLE_MASTER"
        )
        channel_arn = response['ResourceARN']

        # Insert into DB
        cursor.execute(
            "INSERT INTO robots (uuid, robot_name, channel_arn) VALUES (%s, %s, %s);",
            (data.robot_uuid, channel_name, channel_arn)
        )
        conn.commit()

        cursor.execute("SELECT id FROM robots WHERE uuid = %s;", (data.robot_uuid,))
        new_robot_id = cursor.fetchone()[0]

        if not new_robot_id:
            raise HTTPException(status_code=400, detail="Failed to retrieve registered robot")

        return {"robot_id": new_robot_id, "channel_arn": channel_arn}

    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to register {str(e)}")

    finally:
        cursor.close()
        conn.close()

@robots_router.post("/pair")
def pair_robot(data: DevicePair):
    '''Pairs an already registered robot with the user using its robot_id'''
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # Get user_id from cognito_sub
        cursor.execute("SELECT id FROM users WHERE cognito_sub = %s;", (data.user_uuid,))
        user = cursor.fetchone()
        if not user:
            raise HTTPException(status_code=400, detail="User does not exist")

        user_id = user[0]

        # Check if the robot exists using its unique robot_id
        cursor.execute("SELECT id, user_id FROM robots WHERE id = %s;", (data.robot_id,))
        robot = cursor.fetchone()

        # If the robot is not found, return an error (must be pre-registered)
        if not robot:
            raise HTTPException(status_code=404, detail="Robot not found. Ensure it is registered first.")

        robot_id, existing_user_id = robot

        # Check if the robot is already paired with another user
        if existing_user_id and existing_user_id != user_id:
            raise HTTPException(status_code=400, detail="Robot is already paired with another user. Unpair first.")

        # Update robot pairing if unpaired
        if existing_user_id is None:
            cursor.execute("UPDATE robots SET user_id = %s WHERE id = %s;", (user_id, data.robot_id))
            conn.commit()
            return {"message": "Robot successfully paired.", "robot_id": data.robot_id, "user_id": user_id}

        return {"message": "Robot is already paired with this user.", "robot_id": data.robot_id, "user_id": user_id}

    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=400, detail=f"Failed to pair robot: {str(e)}")

    finally:
        cursor.close()
        conn.close()

@robots_router.post("/unpair")
def unpair_robot(data: DevicePair):
    pass

@robots_router.post("/qrcode/{robot_id}")
def generate_qr_code(robot_id: str):
    '''Generates a QR code for the robot to be scanned'''
    return {"robot_id": robot_id, "qrcode": "https://example.com/qrcode.png"}