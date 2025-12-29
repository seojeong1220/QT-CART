from fastapi import FastAPI, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
import models, database

from read_weight import read_cart_weight


app = FastAPI()
models.Base.metadata.create_all(bind=database.engine)

def get_db():
    db = database.SessionLocal()
    try:
        yield db
    finally:
        db.close()

@app.post("/items/", response_model=models.ItemSchema)
def create_item(item: models.ItemCreate, db: Session = Depends(get_db)):
    db_item = models.Item(
        name=item.name, 
        price=item.price, 
        stock=item.stock, 
        weight=item.weight
    )
    db.add(db_item)
    db.commit()
    db.refresh(db_item)
    return db_item

@app.get("/items/{item_id}", response_model=models.ItemSchema)
def read_item(item_id: int, db: Session = Depends(get_db)):
    item = db.query(models.Item).filter(models.Item.id == item_id).first()
    
    if item is None:
        raise HTTPException(status_code=404, detail="Item not found")
    return item

@app.get("/items", response_model=List[models.ItemSchema])
def read_all_items(db: Session = Depends(get_db)):
    return db.query(models.Item).all()

@app.get("/cart/weight")
def get_cart_weight():
    return {
        "cart_weight": read_cart_weight(),
        "unit": "g"
    }

    