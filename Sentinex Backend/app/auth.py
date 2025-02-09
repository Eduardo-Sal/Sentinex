# Auth API used for testing with postman swift has its own AWS SDK for authentication
# However this will use Cognito JWT for authentication

from fastapi import APIRouter, HTTPException
import jwt
import datetime

router = APIRouter()

SECRET_KEY = "" # add later but it's the AWS key

@router.post("/register")
def register_user(user: dict):
    pass

@router.post("/login")
def login_user(user: dict):
    pass
