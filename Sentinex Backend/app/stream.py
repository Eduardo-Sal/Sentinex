# WebRTC/Kinesis video streaming

# look into fastapi websocket
from fastapi import APIRouter, HTTPException
import boto3
from config import aws_region, kinesisvideo, connect_db
import traceback


stream_router = APIRouter()

# starts the stream for the robot
# Returned pre-signed url for webrtc stream which also initializes handshake
# make sure to handle errors in the frontend side 
@stream_router.get("/stream-url/{user_uuid}")
def get_robot_stream_url(user_uuid: str):
    conn = connect_db()
    cursor = conn.cursor()
    try:
        # get robot id via user cognito uuid
        cursor.execute("""
            SELECT r.channel_arn
            FROM users u
            JOIN robots r ON u.id = r.user_id
            WHERE u.cognito_sub = %s;
        """,(user_uuid,))
        result = cursor.fetchone()

        if not result:
            raise HTTPException(statuscode = 404, detail="Robot or channel not found")
        
        channel_arn = result[0]

        # get webrtc endpoint
        endpoints = kinesisvideo.get_signaling_channel_endpoint(
            ChannelARN=channel_arn,
            SingleMasterChannelEndpointConfiguration={
                'Protocols':['WSS', 'HTTPS'],
                'Role': 'VIEWER'
            }
        )
        endpoint_wss = [e['ResourceEndpoint'] for e in endpoints['ResourceEndpointList'] if e['Protocol'] == 'HTTPS'][0]

        if not endpoint_wss:
            raise HTTPException(status_code=500, detail="WSS endpoint not found")

        # Get ICE server config (not a signed URL like S3)
        signaling_client = boto3.client(
            'kinesis-video-signaling',
            endpoint_url=endpoint_wss,
            region_name=aws_region
        )
        ice_response = signaling_client.get_ice_server_config(ChannelARN=channel_arn)

        return {
            "channel_arn": channel_arn,
            "wss_endpoint": endpoint_wss,
            "ice_servers": ice_response
        }

    except Exception as e:
        print("Traceback for error:")
        print(traceback.format_exc())  # logs full stack trace
        raise HTTPException(status_code=500, detail=f"Failed to fetch stream info: {str(e)}")

    finally:
        cursor.close()
        conn.close()