import os
import boto3
import pymysql
from dotenv import load_dotenv

# environment vars
load_dotenv("credentials.env")

# AWS config
s3 = boto3.client("s3")
aws_region = os.getenv("AWS_REGION")
user_pool_id = os.getenv("USER_POOL_ID")
app_client_id = os.getenv("APP_CLIENT_ID")
s3_bucket = os.getenv("S3_BUCKET", "sentinexbucket")

# MySQL config
db_host = os.getenv("DB_HOST")
db_user = os.getenv("DB_USER")
db_pass = os.getenv("DB_PASS")
db_name = os.getenv("DB_NAME")

# AWS Clients
s3 = boto3.client("s3")
cognito = boto3.client("cognito-idp", region_name=aws_region)

# Function to connect to MySQL
def connect_db():
    return pymysql.connect(host=db_host, user=db_user, password=db_pass, database=db_name)