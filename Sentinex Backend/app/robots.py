# Robot registration & functionality like controlling it and streams

# Optimize later by including HTTPException and depends
from fastapi import APIRouter
import boto3

router = APIRouter()
dynamodb = boto3.resource("") # add later but it's dynamo
robots_table = dynamodb.Table("") # add later table schema willbe "Robots"

@router.post("/register")
def register_robot(data: dict):
    '''Registers robot to a user if scanned with QR code or manually typed'''
    robots_table.put_item(Item={
        "robot_id": data["robot_id"],
        "user_id": data["user_id"],
        "public_key": data["public_key"],
    })
    return {"message": "Robot registered successfully"}

# To be tested later
@router.post("/{robot_id}/move")
def move_robot(robot_id: str, command: dict):
    '''sends commands to the robot to move from the mobile app'''
    return {"robot_id": robot_id, "command": command}