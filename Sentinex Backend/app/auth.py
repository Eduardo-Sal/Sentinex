# Auth API used for testing with postman swift has its own AWS SDK for authentication
# However this will use Cognito JWT for authentication

from fastapi import APIRouter, HTTPException
import jwt
import os
import boto3
from dotenv import load_dotenv
from pydantic import BaseModel
import pymysql

load_dotenv("credentials.env")
auth_router = APIRouter()

# Load AWS credentials using the environment file, we need s3 bucket, cognito, and mysql
# AWS credentials   
s3= boto3.client("s3")
s3_bucket = os.getenv("S3_BUCKET","sentinexbucket")
aws_region = os.getenv("AWS_REGION")
user_pool_id = os.getenv("USER_POOL_ID")
app_client_id = os.getenv("APP_CLIENT_ID")
# Mysql credentials
db_host = os.getenv("DB_HOST")
db_user = os.getenv("DB_USER")
db_pass = os.getenv("DB_PASS")
db_name = os.getenv("DB_NAME")

cognito = boto3.client("cognito-idp",region_name = aws_region)

# Connects to MYSQL
def connect_db():
    return pymysql.connect(host=db_host,user=db_user,password=db_pass,database=db_name)

# Use Pydantic model 
# Sign up
class UserCredentials(BaseModel):
    email: str
    password: str
# Confirm Email
class ConfirmEmail(BaseModel):
     email: str
     code: str
# Delete account
class DeleteAccount(BaseModel):
     user_refresh_token: str


def delete_user_s3_folder(user_id):
    """Deletes all objects under a specific prefix folder in an S3 bucket."""
    prefix = f"{user_id}/"
    try:
        # List all objects under the prefix
        objects = s3.list_objects_v2(Bucket=s3_bucket, Prefix=prefix)

        # Check if there are objects to delete
        if 'Contents' in objects:
            for obj in objects['Contents']:
                key = obj.get('Key')
                if not key:
                    print("Encountered an object with no key, skipping.")
                    continue
                s3.delete_object(Bucket=s3_bucket, Key=key)
                print(f"Deleted: {key}")
        else:
            print("No objects found")

        print(f"Successfully deleted all objects under {prefix}")

    except Exception as e:
        print(f"Failed to delete objects: {e}")


          
def create_user_s3_folders(user_id):
    try:
        # Create a base folder, and subfolders which are named screenshot and video
        base_folder = f"{user_id}/"
        subfolders = ["screenshot/","clips/"]
        for folder in subfolders:
            key = base_folder + folder
            s3.put_object(Bucket=s3_bucket,Key=key)
        print(f"Folder created:{base_folder}")
    except Exception as e:
        print(f"Failed to create folder for user {base_folder} error: {e}")

# Handle edge cases later which is if doesn't exists in cognito but exists in mysql
@auth_router.post("/register")
def register_user(user: UserCredentials):
    '''
    Registers account using cognito, mysql, and create S3 bucket associated with cognito UUID bucket
    '''
    try:
        response = cognito.sign_up(
            ClientId = app_client_id,
            Username = user.email,
            Password = user.password,
            UserAttributes=[{"Name": "email", "Value": user.email}],
        )

        # Get Cognito user ID (`sub`)
        cognito_sub = response["UserSub"]
        print(f"Cognito User Created: {cognito_sub}")

        # Store user details in MySQL
        conn = connect_db()
        cursor = conn.cursor()
        query = "INSERT INTO users (cognito_sub, email) VALUES (%s, %s);"
        cursor.execute(query, (cognito_sub, user.email))
        conn.commit()
        cursor.close()
        conn.close()

        # Create S3 folder
        create_user_s3_folders(cognito_sub) 

        return {"message": "User registered successfully!", "cognito_id": cognito_sub}

    except Exception as e:
                raise HTTPException(status_code=400, detail=f"Registration failed: {str(e)}")
    

@auth_router.post("/verify-email")
def verify_user_email(data: ConfirmEmail):
     '''
     Verify user email using cognito
     '''
     try:
          response = cognito.confirm_sign_up(
               ClientId = app_client_id,
               Username = data.email,
               ConfirmationCode = data.code
          )
          return {"Email Verified!"}
     except Exception as e:
          raise HTTPException(status_code=400, details=f"Failed to verify email: {str(e)}")

@auth_router.post("/login")
def login_user(user: UserCredentials):
    '''logs user in using cognito authentication function'''
    try:
         response = cognito.initiate_auth(
              ClientId = app_client_id,
              AuthFlow = "USER_PASSWORD_AUTH",
              AuthParameters = {
                   "USERNAME": user.email,
                   "PASSWORD": user.password
              }
         )
         auth_result = response["AuthenticationResult"]
         return {
              "access_token":auth_result["AccessToken"],
              "id_token": auth_result["IdToken"],
              "refresh_token": auth_result["RefreshToken"],
              "expires_in": auth_result["ExpiresIn"]
         }
    except Exception as e:
         raise HTTPException(status_code=400, details=f"Failed to login: {str(e)}")

@auth_router.delete("/delete-account")
def delete_user(request: DeleteAccount):
    '''
    Deletes user account from Cognito and MySQL
    '''
    try:
         # Refresh user token
         refresh_response = cognito.initiate_auth(
              ClientId = app_client_id,
              AuthFlow = "REFRESH_TOKEN_AUTH",
              AuthParameters = {"REFRESH_TOKEN": request.user_refresh_token}
         )
         new_access_token = refresh_response["AuthenticationResult"]["AccessToken"]
         # Delete Account
         cognito.delete_user(
              AccessToken = new_access_token
         )
         # Decode Access Token to Get Cognito `sub`
         decoded_token = jwt.decode(new_access_token, options={"verify_signature": False})
         cognito_sub = decoded_token.get("sub")

         if not cognito_sub:
            raise HTTPException(status_code=400, detail="Failed to extract Cognito user ID.")

        # Deletes from MySQL
         conn = connect_db()
         cursor = conn.cursor()
         
         # fetch user_id and cognito_sub qhich is UUID
         query = "SELECT id, cognito_sub FROM users WHERE cognito_sub = %s;"
         cursor.execute(query, (cognito_sub,))
         user_data = cursor.fetchone()


         if not user_data:
            raise HTTPException(status_code=404, detail="User not found in database.")
         
         user_id, cognito_uuid = user_data
         delete_user_s3_folder(cognito_uuid)


         cursor.execute("DELETE FROM robots WHERE user_id = %s;", (user_id,))
         cursor.execute("DELETE FROM alerts WHERE robot_id IN (SELECT id FROM robots WHERE user_id = %s);", (user_id,))
         cursor.execute("DELETE FROM screenshots WHERE robot_id IN (SELECT id FROM robots WHERE user_id = %s);", (user_id,))
         cursor.execute("DELETE FROM clips WHERE robot_id IN (SELECT id FROM robots WHERE user_id = %s);", (user_id,))
         cursor.execute("DELETE FROM users WHERE id = %s;", (user_id,))

         conn.commit()
         cursor.close()
         conn.close()


         return {"Account successfully deleted"}
    
    except Exception as e:
         raise HTTPException(status_code=400, details = f"Failed to delete user account: {str(e)}")
         

# TODO later forgot password
@auth_router.post("/forgot-password")
def forgot_password(user: dict):
     pass

