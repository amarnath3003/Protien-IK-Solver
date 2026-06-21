from fastapi import FastAPI, WebSocket
import uvicorn

app = FastAPI()

@app.websocket('/ws')
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    await websocket.send_text('Hello')
    await websocket.close()

if __name__ == '__main__':
    uvicorn.run(app, host='127.0.0.1', port=8001)
