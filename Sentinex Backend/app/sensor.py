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


sensor_router = APIRouter()


@sensor_router.post("/{robot_id}/temperature")
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
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

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

@sensor_router.get("/{robot_id}/temperature")
def get_temperature_data(robot_id: int):
    '''
    Returns all temperature readings and the average temperature for the robot.
    '''
    conn = connect_db()
    cursor = conn.cursor()
    try:
        cursor.execute(
            "SELECT temperature_readings FROM robots WHERE id = %s;",
            (robot_id,)
        )
        result = cursor.fetchone()
        if not result:
            raise HTTPException(status_code=404, detail="Robot not found")

        readings = json.loads(result[0] or "[]")
        temp_values = [item[1] for item in readings if isinstance(item, list) and len(item) == 2]
        avg_temperature = round(sum(temp_values) / len(temp_values), 2) if temp_values else None

        return {
            "robot_id": robot_id,
            "temperature_readings": readings,
            "average_temperature": avg_temperature
        }

    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to retrieve temperature data: {str(e)}")
    finally:
        cursor.close()
        conn.close()

@sensor_router.post("/{robot_id}/sound")
def update_sound_anomaly(robot_id: int, payload: SoundPayload):
    '''
    Adds a (timestamp, db_level) tuple to sound_anomalies column and inserts a sound notification.
    '''
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # Get current sound anomalies
        cursor.execute("SELECT sound_anomalies, user_id FROM robots WHERE id = %s;", (robot_id,))
        result = cursor.fetchone()
        if not result:
            raise HTTPException(status_code=404, detail="Robot not found")

        anomalies = json.loads(result[0] or "[]")
        user_id = result[1]

        # Append new anomaly with timestamp (use local time in readable format)
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        anomalies.append([timestamp, payload.db_level])

        # Insert notification with db_level in its own column and local time timestamp
        cursor.execute(
            """
            INSERT INTO notifications (user_id, robot_id, s3_filename, timestamp, media_type, event_type, db_level)
            VALUES (%s, %s, %s, %s, %s, %s, %s);
            """,
            (user_id, robot_id, '', timestamp, 'sound', 'sound_anomaly', payload.db_level)
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

@sensor_router.get("/{robot_id}/sound")
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

@sensor_router.get("/{robot_id}/data")
def get_sensor_data(robot_id: int):
    conn = connect_db()
    cursor = conn.cursor()
    try:
        cursor.execute(
            "SELECT temperature_readings, sound_anomalies FROM robots WHERE id = %s;",
            (robot_id,)
        )
        result = cursor.fetchone()
        if not result:
            raise HTTPException(status_code=404, detail="Robot not found")

        temp_readings = json.loads(result[0] or "[]")
        sound_anomalies = json.loads(result[1] or "[]")
        average_temp = sum(r[1] for r in temp_readings) / len(temp_readings) if temp_readings else 0

        return {
            "robot_id": robot_id,
            "temperature_readings": temp_readings,
            "average_temperature": average_temp,
            "sound_anomalies": sound_anomalies,
            "sound_anomaly_count": len(sound_anomalies)
        }
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to fetch sensor data: {str(e)}")
    finally:
        cursor.close()
        conn.close()