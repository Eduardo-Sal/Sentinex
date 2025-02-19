# Main entry point (FASTAPI app)
from fastapi import FastAPI
from auth import auth_router
from robots import robots_router
#from screenshots import router as screenshots_router
#from clips import router as clips_router
#from stream import router as stream_router
import uvicorn 

# Initialize the FastAPI app
app = FastAPI()

# Register API Routes for backend
app.include_router(auth_router, prefix="/api/auth")
app.include_router(robots_router, prefix="/api/robots")
#app.include_router(screenshots_router, prefix="/api/screenshots")
#app.include_router(clips_router, prefix="/api/clips")
#app.include_router(stream_router, prefix="/api/stream")

if __name__ == "__main__":
    uvicorn.run("main:app",host="127.0.0.1", port=8000, reload=True)

