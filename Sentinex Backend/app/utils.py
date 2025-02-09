# Handles utility functions like S3 and JWT validation

import jwt
import boto3
from fastapi import HTTPException, Depends
from config import SECRET_KEY, AWS_REGION, S3_BUCKET

s3 = boto3.client("")

# validates the JWT token and extracts user_id for image
def validate_jwt(token: str):
    pass

# generates the presigned url for the image retrival from s3
def get_presigned_s3_url(file_key: str, expiration= 60):
    pass