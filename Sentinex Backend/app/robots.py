# Robot registration & functionality like controlling it and streams
# Optimize later by including HTTPException and depends

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, constr, conint
from config import  connect_db, kinesisvideo
from typing import Optional
from datetime import datetime, timedelta
import json
from datetime import datetime

class TemperaturePayload(BaseModel):
    temperature: float

class SoundPayload(BaseModel):
    db_level: float

class DevicePair(BaseModel):
    user_uuid: constr(min_length=36, max_length=36)
    robot_id: conint(gt=0)

class DeviceUnpair(BaseModel):
    user_uuid: constr(min_length=36, max_length=36)

class RobotRegistration(BaseModel):
    robot_uuid: constr(min_length=36, max_length=36)

class PairingRequest(BaseModel):
    pairing_enabled: bool
    pairing_duration_minutes: Optional[int] = 3 # defaults to 3 minutes if not specified

robots_router = APIRouter()

@robots_router.patch("/{robot_id}/discovery-mode")
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

        # Check if the robot exists and retrieve pairing status
        cursor.execute("""
            SELECT id, user_id, pairing_enabled, pairing_expires_at 
            FROM robots 
            WHERE id = %s;
        """, (data.robot_id,))
        robot = cursor.fetchone()

        if not robot:
            raise HTTPException(status_code=404, detail="Robot not found. Ensure it is registered first.")

        robot_id, existing_user_id, pairing_enabled, pairing_expires_at = robot

        # Check pairing enabled flag
        if not pairing_enabled:
            raise HTTPException(status_code=403, detail="Pairing not enabled on this robot.")

        # Check expiration
        if pairing_expires_at and datetime.utcnow() > pairing_expires_at:
            raise HTTPException(status_code=403, detail="Pairing window expired. Press button again to re-enable.")

        # Check if robot is already paired with a different user
        if existing_user_id and existing_user_id != user_id:
            raise HTTPException(status_code=400, detail="Robot is already paired with another user. Unpair first.")

        # Pair the robot to the user
        cursor.execute("""
            UPDATE robots 
            SET user_id = %s, pairing_enabled = 0, pairing_expires_at = NULL 
            WHERE id = %s;
        """, (user_id, robot_id))
        conn.commit()

        return {
            "message": "Robot successfully paired.",
            "robot_id": robot_id,
            "user_id": user_id
        }

    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=400, detail=f"Failed to pair robot: {str(e)}")

    finally:
        cursor.close()
        conn.close()

@robots_router.post("/unpair")
def unpair_robot(data: DeviceUnpair):
    '''Unpairs robot from requested user uuid'''
    conn = connect_db()
    cursor = conn.cursor()
    try:
        cursor.execute("SELECT id FROM users WHERE cognito_sub = %s;", (data.user_uuid,))
        user = cursor.fetchone()

        # If user doesn't exist raise error
        if not user:
            raise HTTPException(status_code=400, detail="User does not exist")
        
        user_id = user[0]

        # Check if robot exists and retrieve robot_id for unpairing
        cursor.execute("""
            UPDATE robots
            SET user_id = NULL
            WHERE user_id = %s;
        """, (user_id,))
        conn.commit()

        return {"message": "Robot unpaired successfully", "user_id": user_id}

    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=400, detail=f"Failed to unpair {e}")
    finally:
        cursor.close()
        conn.close()

@robots_router.post("/{robot_id}/temperature")
def update_temperature(robot_id: int, payload: TemperaturePayload):
    '''
    Adds a (timestamp, temperature) tuple to the robot in circular fashion using temperature_index.
    Backend generates the timestamp.
    '''
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # Get current readings + index
        cursor.execute(
            "SELECT temperature_readings, temperature_index FROM robots WHERE id = %s;",
            (robot_id,)
        )
        result = cursor.fetchone()
        if not result:
            raise HTTPException(status_code=404, detail="Robot not found")

        readings = json.loads(result[0] or "[]")
        index = result[1] or 0

        # Create timestamp
        timestamp = datetime.utcnow().isoformat() + "Z"

        # Expand array if not full
        if len(readings) < 12:
            readings.append([timestamp, payload.temperature])
        else:
            # Overwrite at index
            readings[index % 12] = [timestamp, payload.temperature]

        # Update index
        new_index = (index + 1) % 12

        # Save back to DB
        cursor.execute(
            "UPDATE robots SET temperature_readings = %s, temperature_index = %s WHERE id = %s;",
            (json.dumps(readings), new_index, robot_id)
        )
        conn.commit()

        return {
            "success": True,
            "robot_id": robot_id,
            "updated_readings": readings,
            "next_index": new_index
        }

    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=400, detail=f"Failed to update temperature: {str(e)}")
    finally:
        cursor.close()
        conn.close()


@robots_router.post("/{robot_id}/sound")
def update_sound_anomaly(robot_id: int, payload: SoundPayload):
    '''
    Adds a (timestamp, db_level) tuple to sound_anomalies column.
    '''
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # Get current sound anomalies
        cursor.execute("SELECT sound_anomalies FROM robots WHERE id = %s;", (robot_id,))
        result = cursor.fetchone()
        if not result:
            raise HTTPException(status_code=404, detail="Robot not found")

        anomalies = json.loads(result[0] or "[]")

        # Append new anomaly with timestamp
        timestamp = datetime.utcnow().isoformat() + "Z"
        anomalies.append([timestamp, payload.db_level])

        # Save back to DB
        cursor.execute(
            "UPDATE robots SET sound_anomalies = %s WHERE id = %s;",
            (json.dumps(anomalies), robot_id)
        )
        conn.commit()

        # Optionally insert a notification
        cursor.execute(
            """
            INSERT INTO notifications (user_id, robot_id, s3_filename, media_type)
            SELECT user_id, %s, %s, %s FROM robots WHERE id = %s;
            """,
            (robot_id, 'sound_anomaly', 'sound', robot_id)
        )
        conn.commit()

        return {
            "success": True,
            "robot_id": robot_id,
            "updated_anomalies": anomalies
        }

    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=400, detail=f"Failed to update sound anomaly: {str(e)}")
    finally:
        cursor.close()
        conn.close()


@robots_router.get("/{robot_id}/sound")
def get_sound_anomalies(robot_id: int):
    '''
    Retrieves all sound anomalies for the robot.
    '''
    conn = connect_db()
    cursor = conn.cursor()
    try:
        cursor.execute("SELECT sound_anomalies FROM robots WHERE id = %s;", (robot_id,))
        result = cursor.fetchone()
        if not result:
            raise HTTPException(status_code=404, detail="Robot not found")

        anomalies = json.loads(result[0] or "[]")
        return {"robot_id": robot_id, "sound_anomalies": anomalies}

    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to retrieve sound anomalies: {str(e)}")
    finally:
        cursor.close()
        conn.close()