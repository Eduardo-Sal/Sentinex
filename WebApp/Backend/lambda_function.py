# AWS Lambda handler

from mangum import Mangum
# import fastapi from the app folder
from app.main import app

# AWS Lambda handler
handler = Mangum(app)

