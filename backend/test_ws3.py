from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)

@app.websocket('/ws/solve')
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    await websocket.send_text('Hello')
    await websocket.close()

if __name__ == '__main__':
    uvicorn.run(app, host='127.0.0.1', port=8005)
