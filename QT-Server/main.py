import uvicorn
from fastapi import FastAPI, Depends, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse 
from sqlalchemy.orm import Session
from typing import List
from pydantic import BaseModel
import models, database
import logging
from datetime import datetime
import dashboard
from read_weight import read_cart_weight

CART_ITEMS = []
EXPECTED_WEIGHT = 0.0

app = FastAPI()
models.Base.metadata.create_all(bind=database.engine)

# 삭제해라
def read_cart_weight():
    return 300.0

# 라우터 및 정적 파일 설정
app.include_router(dashboard.router)
app.mount("/static", StaticFiles(directory="static"), name="static")

# 상태 저장
bot_state_store = {
    "status": "OFFLINE",
    "last_updated": datetime.now(),
    "battery": 0.0,
    "speed": 0.0
}

# 데이터 수신 모델 (ROS -> Server)
class BotReport(BaseModel):
    status: str
    battery: float = 0.0
    speed: float = 0.0

def get_db():
    db = database.SessionLocal()
    try: yield db
    finally: db.close()

# --- 참고 코드의 무게 검증 로직 이식 ---
def is_movable(expected, real):
    if expected <= 0:
        return True
    diff_ratio = abs(real - expected) / expected
    return diff_ratio <= 0.05

# --- 페이지 라우팅 ---
@app.get("/dashboard")
async def dashboard_view(): return FileResponse("templates/dashboard.html")

@app.get("/inventory")
async def inventory_view(): return FileResponse("templates/inventory.html")

# --- 무게 영점 조절 (Tare) ---
@app.post("/cart/tare")
def tare_cart():
    global EXPECTED_WEIGHT
    EXPECTED_WEIGHT = read_cart_weight()
    return {
        "message": "Cart tared",
        "expected_weight": EXPECTED_WEIGHT
    }

# --- 상품 관리 API ---
@app.post("/items/", response_model=models.ItemSchema)
def create_item(item: models.ItemCreate, db: Session = Depends(get_db)):
    db_item = models.Item(
        name=item.name, price=item.price, stock=item.stock, 
        weight=item.weight, expiry_date=item.expiry_date
    )
    db.add(db_item)
    db.commit()
    db.refresh(db_item)
    return db_item

@app.get("/items", response_model=List[models.ItemSchema])
def read_all_items(db: Session = Depends(get_db)):
    return db.query(models.Item).all()

# --- 장바구니 ---
@app.post("/cart/add/{item_id}")
def add_item_to_cart(item_id: int, db: Session = Depends(get_db)):
    global EXPECTED_WEIGHT, CART_ITEMS

    item = db.query(models.Item).filter(models.Item.id == item_id).first()
    if not item: raise HTTPException(status_code=404, detail="Item not found")
    
    # 리스트에 추가
    CART_ITEMS.append({
        "id": item.id, 
        "name": item.name, 
        "price": item.price, 
        "weight": item.weight
    })
    
    EXPECTED_WEIGHT += item.weight
    
    real_weight = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real_weight)
    diff = abs(real_weight - EXPECTED_WEIGHT)

    return {
        "msg": "Added",
        "action": "add",
        "current_cart": {
            "items": CART_ITEMS,
            "total_price": sum(i["price"] for i in CART_ITEMS),
            "count": len(CART_ITEMS)
        },
        "weight_info": {
            "expected": EXPECTED_WEIGHT,
            "real": real_weight,
            "diff": diff,
            "movable": movable
        }
    }

# 상품 제거
@app.post("/cart/remove/{item_id}")
def remove_item_from_cart(item_id: int):
    global EXPECTED_WEIGHT, CART_ITEMS

    # ID로 인덱스 찾기
    target_idx = next((i for i, item in enumerate(CART_ITEMS) if item["id"] == item_id), None)
    
    if target_idx is not None:
        removed = CART_ITEMS.pop(target_idx)
        EXPECTED_WEIGHT -= removed["weight"]
        
        if EXPECTED_WEIGHT < 0: EXPECTED_WEIGHT = 0.0

        # 무게 검증
        real_weight = read_cart_weight()
        movable = is_movable(EXPECTED_WEIGHT, real_weight)
        diff = abs(real_weight - EXPECTED_WEIGHT)

        return {
            "msg": "Removed",
            "action": "remove",
            "current_cart": {
                "items": CART_ITEMS,
                "total_price": sum(i["price"] for i in CART_ITEMS),
                "count": len(CART_ITEMS)
            },
            "weight_info": {
                "expected": EXPECTED_WEIGHT,
                "real": real_weight,
                "diff": diff,
                "movable": movable
            }
        }
        
    raise HTTPException(status_code=404, detail="Item not in cart")

# 장바구니 리셋
@app.post("/cart/reset")
def reset_cart():
    global CART_ITEMS, EXPECTED_WEIGHT
    CART_ITEMS = []
    EXPECTED_WEIGHT = 0.0
    return {"msg": "Reset done"}

# 장바구니 상태 조회 
@app.get("/cart/status")
def get_cart_full_status():
    global EXPECTED_WEIGHT, CART_ITEMS
    
    real = read_cart_weight()
    movable = is_movable(EXPECTED_WEIGHT, real)
    diff = real - EXPECTED_WEIGHT
    
    # 대시보드 UI에 표시할 상태 텍스트 결정
    status_text = "NORMAL" if movable else "WARNING_WEIGHT_MISMATCH"
    
    total_price = sum(item["price"] for item in CART_ITEMS)
    
    return {
        "real_weight": real,
        "expected_weight": EXPECTED_WEIGHT,
        "weight_diff": diff,
        "cart_items": CART_ITEMS,
        "total_price": total_price,
        "total_count": len(CART_ITEMS),
        "system_status": status_text, # 대시보드 경고등용
        "movable": movable
    }

# --- 결제 및 매출 ---
@app.post("/cart/pay")
def pay_cart(db: Session = Depends(get_db)):
    if not CART_ITEMS:
        raise HTTPException(status_code=400, detail="장바구니가 비어있습니다.")

    total_price = sum(item["price"] for item in CART_ITEMS)

    new_pay = models.Pay(
        total_price=total_price,
        created_at=datetime.now()
    )
    db.add(new_pay)
    db.commit()

    reset_cart() # 결제 후 장바구니 비우기

    return {"msg": "결제 완료", "amount": total_price}

@app.get("/payments", response_model=List[models.PaySchema])
def get_payment_history(db: Session = Depends(get_db)):
    return db.query(models.Pay).order_by(models.Pay.created_at.desc()).all()

# --- ROS 봇 통신 API ---
@app.post("/dashboard/bot/report")
def report_bot_status(report: BotReport):
    bot_state_store["status"] = "ONLINE"
    bot_state_store["last_updated"] = datetime.now()
    bot_state_store["battery"] = report.battery
    bot_state_store["speed"] = report.speed
    return {"msg": "updated"}

@app.get("/dashboard/bot/check")
def check_bot_status():
    time_diff = (datetime.now() - bot_state_store["last_updated"]).total_seconds()
    if time_diff > 5:
        bot_state_store["status"] = "OFFLINE"
        bot_state_store["speed"] = 0.0

    return {
        "bot_status": bot_state_store["status"],
        "battery": bot_state_store["battery"],
        "speed": bot_state_store["speed"]
    }

@app.on_event("startup")
def startup_event():
    # 서버 재시작 시 상태 초기화
    reset_cart()
    print(">>> System Started: Cart Reset Done.")

# --- 로그 필터 ---
class EndpointFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        return ("/bot/check" not in record.getMessage() and
                "/bot/report" not in record.getMessage() and
                "/cart/status" not in record.getMessage() 
        )

logging.getLogger("uvicorn.access").addFilter(EndpointFilter())

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
