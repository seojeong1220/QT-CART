from fastapi import APIRouter
from pydantic import BaseModel
from datetime import datetime
from typing import Optional

router = APIRouter(
    prefix="/dashboard",
    tags=["Dashboard"]
)

# 로봇의 마지막 응답 시간을 저장
last_bot_response_time = None 
BOT_TIMEOUT_SEC = 5 # 5초 이내에 응답이 없으면 연결 끊김

# --- DTO ---
class BotStatusReport(BaseModel):
    status: str  

class BotConnectionInfo(BaseModel):
    bot_status: str        
    last_update: Optional[str] 

# 로봇 -> 서버: 현재 상태 보고 
@router.post("/bot/report")
def report_bot_status(report: BotStatusReport):
    global last_bot_response_time
    # 로봇으로부터 리포트를 받은 시각을 기록
    last_bot_response_time = datetime.now()
    return {"msg": "status report received"}

# 서버 -> 프론트엔드: 로봇 연결 정보 제공
@router.get("/bot/check", response_model=BotConnectionInfo)
def check_bot_connection():
    global last_bot_response_time
    
    current_state = "OFFLINE"
    last_time_str = "-"
    
    if last_bot_response_time:
        elapsed = (datetime.now() - last_bot_response_time).total_seconds()
        
        if elapsed < BOT_TIMEOUT_SEC:
            current_state = "ONLINE"
            
        last_time_str = last_bot_response_time.strftime("%H:%M:%S")

    return {
        "bot_status": current_state,
        "last_update": last_time_str
    }