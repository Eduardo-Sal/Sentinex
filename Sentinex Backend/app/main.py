from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from auth import auth_router
from robots import robots_router
from images import images_router
from clips import clips_router
from notifications import notifications_router
from stream import stream_router
#import uvicorn

# Initialize the FastAPI app
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],  # Allow only frontend origin
    allow_credentials=True,
    allow_methods=["*"],  # Allow all HTTP methods
    allow_headers=["*"],  # Allow all headers
)

# Register API Routes for backend
app.include_router(auth_router, prefix="/api/auth")
app.include_router(robots_router, prefix="/api/robots")
app.include_router(images_router, prefix="/api/images")
app.include_router(clips_router, prefix="/api/clips")
app.include_router(notifications_router, prefix="/api/notifications")
app.include_router(stream_router, prefix="/api/stream")

# Run the FastAPI server
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True)