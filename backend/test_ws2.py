from fastapi import FastAPI, WebSocket
import uvicorn
import asyncio

app = FastAPI()

@app.websocket('/ws/solve')
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    await websocket.send_text('Hello')
    await websocket.close()

if __name__ == '__main__':
    uvicorn.run(app, host='127.0.0.1', port=8003)
