import asyncio
import logging
import json
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

conn = 0

async def do_cmd(cmd):
    global conn
    print("Doing: " + cmd)
    await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {"api_id": SPORT_CMD[cmd]}
        )


async def say_hello():
    global conn
    await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {"api_id": SPORT_CMD["Hello"]}
        )

    
async def move(x, y, z):
    global conn
    print("Moving forward...")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"], 
        {
            "api_id": SPORT_CMD["Move"],
            "parameter": {"x": x, "y": y, "z": z}
        }
    )
    


    
async def main():
    global conn
    try:
        # Choose a connection method (uncomment the correct one)
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D1000P6IAA88B")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)

        # Connect to the WebRTC service.
        await conn.connect()

        finished = False
        while not finished:
            i = int(input("Enter command: "))
            match i:
                case 0:
                    await say_hello()
                    continue
                case 1:
                    await move(0.5, 0, 0)
                    continue
                case 2:
                    await move(-0.5, 0, 0)
                    continue
                case 3:
                    await move(0, 0.5, 0)
                    continue
                case 4:
                    await move(0, -0.5, 0)
                    continue
            key = list(SPORT_CMD.keys())[i -5]
            await do_cmd(key)
                
        # Keep the program running for a while
        await asyncio.sleep(3600)
    
    except ValueError as e:
        # Log any value errors that occur during the process.
        logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # Handle Ctrl+C to exit gracefully.
        print("\nProgram interrupted by user")
        sys.exit(0)