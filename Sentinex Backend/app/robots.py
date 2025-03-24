# Robot registration & functionality like controlling it and streams
# Optimize later by including HTTPException and depends

from fastapi import APIRouter, HTTPException
from dotenv import load_dotenv
from pydantic import BaseModel
from config import cognito, connect_db, app_client_id, s3_bucket, s3, aws_region, user_pool_id

class DevicePair(BaseModel):
    user_uuid: str
    robot_id: str

class RobotRegistration(BaseModel):
    robot_uuid: str

robots_router = APIRouter()

@robots_router.post("/register")
def register_robot(data: RobotRegistration):
    '''
    Registers robot to a user
    '''
    # connect to db
    conn = connect_db()
    cursor = conn.cursor()
    try:
    # check if it exists
        cursor.execute("SELECT id from robots WHERE uuid = %s;", (data.robot_uuid,))
        existing_robot = cursor.fetchone()

        if existing_robot:
            return {"robot_id": existing_robot[0]}

    # if it doesn't add it 
        cursor.execute("INSERT into robots (uuid, robot_name) VALUES (%s,%s);",(data.robot_uuid, f"robot-{data.robot_uuid}"))
        conn.commit()

        cursor.execute("SELECT id from robots WHERE uuid = %s;", (data.robot_uuid))
        new_robot_id = cursor.fetchone()[0]

        if not new_robot_id:
            raise HTTPException(status_code = 400, detail = "Failed to retrieve registered robot")
        
        return{"robot_id": new_robot_id}
    
    # get the robot_id and return it to the esp32
    except Exception as e:
        return HTTPException(status_code=400, detail=f"Failed to register {str(e)}")
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

# To be tested later
@robots_router.post("/{robot_id}/move")
def move_robot(robot_id: str, command: dict):
    '''sends commands to the robot to move from the mobile app'''
    return {"robot_id": robot_id, "command": command}


# Test with MQTT both start_control and stop_control
@robots_router.get("/start-control/{robot_id}")
def start_control(robot_id: str):
    '''Starts the control of the robot, essentially starts a websocket'''
    pass

@robots_router.post("/stop-control/{robot_id}")
def stop_control(robot_id: str):
    '''stops the control of the robot, so turns off the websocket'''