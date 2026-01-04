let currentCartId = 1;
let isTestMode = false;

const mapImg = new Image();
mapImg.src = "/static/map601.png"; 

// [Calibration] 화면 모서리 좌표 매핑
// X축: -0.67(좌) ~ 3.34(우) -> 반대일 수도 있으니 테스트하며 확인
// Y축: 1.66(상) ~ -7.4(하)
const MAP_BOUNDS = {
    LEFT_X: -0.67,
    RIGHT_X: 3.34,
    TOP_Y: 1.66,
    BOTTOM_Y: -7.4
};

const allCartsData = {
    // human_x, human_y 데이터 필드 추가
    1: { id: 1, name: "QTCART 1호차", isOnline: false, battery: 0, speed: 0, weight: 0, expected: 0, items: [], status: "OFFLINE", x: 0.0, y: 0.0, human_x: 0.0, human_y: 0.0 },
    2: { id: 2, name: "QTCART 2호차", isOnline: true, battery: 92, speed: 50, weight: 710, expected: 700, items: generateMockItems(3), status: "ONLINE", x: 1.5, y: 1.0 },
    3: { id: 3, name: "QTCART 3호차", isOnline: true, battery: 34, speed: 0, weight: 0, expected: 0, items: [], status: "ONLINE", x: -0.67, y: 1.66 },
    4: { id: 4, name: "QTCART 4호차", isOnline: true, battery: 65, speed: 0, weight: 0, expected: 0, items: [], status: "ONLINE", x: 3.34, y: -7.4 }
};

function generateMockItems(count) {
    const mockDb = [
        {name: "코카콜라 500ml", price: 2000, weight: 500},
        {name: "포카칩 오리지널", price: 1500, weight: 100},
        {name: "제주 삼다수 2L", price: 1000, weight: 2000},
        {name: "신라면 멀티팩", price: 4500, weight: 600}
    ];
    let items = [];
    for(let i=0; i<count; i++) {
        const rand = mockDb[Math.floor(Math.random() * mockDb.length)];
        items.push({...rand, id: 100+i});
    }
    return items;
}

function applyTestCoords() {
    isTestMode = true; 
    const xVal = parseFloat(document.getElementById('testX').value);
    const yVal = parseFloat(document.getElementById('testY').value);
    
    allCartsData[1].isOnline = true;
    allCartsData[1].status = "ONLINE";
    allCartsData[1].x = xVal;
    allCartsData[1].y = yVal;
    
    drawMap(); 
}

async function updateDashboard() {
    const apiBadge = document.getElementById('apiBadge');

    try {
        const botRes = await fetch('/dashboard/bot/check');
        const botData = await botRes.json();
        
        let cartData = null;
        try { const cRes = await fetch('/cart/status'); if(cRes.ok) cartData = await cRes.json(); } catch(e) {}

        apiBadge.className = 'status-badge on';
        apiBadge.innerHTML = '<span class="dot"></span> 서버 연결됨';

        const c1 = allCartsData[1]; 

        if (!isTestMode) {
            if(botData.bot_status === 'ONLINE') {
                c1.isOnline = true;
                c1.status = "ONLINE";
                c1.battery = parseInt(botData.battery);
                c1.speed = Math.abs(Math.round(botData.speed * 100));

                // [수정] 로봇 좌표 & 사람 좌표 업데이트
                c1.x = botData.x !== undefined ? botData.x : 0.0;
                c1.y = botData.y !== undefined ? botData.y : 0.0;
                c1.human_x = botData.human_x !== undefined ? botData.human_x : 0.0;
                c1.human_y = botData.human_y !== undefined ? botData.human_y : 0.0;
                
                // 입력창 동기화
                const tx = document.getElementById('testX');
                const ty = document.getElementById('testY');
                if(tx && ty) {
                    tx.value = c1.x;
                    ty.value = c1.y;
                }
                
                if(cartData) {
                    c1.weight = cartData.real_weight;
                    c1.expected = cartData.expected_weight;
                    c1.items = cartData.cart_items || [];
                    
                    if(cartData.system_status === "WARNING_WEIGHT_MISMATCH") {
                        c1.status = "WARNING";
                    } else {
                        c1.status = "ONLINE";
                    }
                }
            } else {
                c1.isOnline = false;
                c1.status = "OFFLINE";
                c1.speed = 0;
            }
        }

    } catch (err) {
        apiBadge.className = 'status-badge off';
        apiBadge.innerHTML = '<span class="dot"></span> 서버 끊김';
        if(!isTestMode) {
            allCartsData[1].isOnline = false;
            allCartsData[1].status = "OFFLINE";
        }
    }

    updateMockData(2);
    // updateMockData(3); 
    // updateMockData(4); 

    renderGrid();        
    renderDetailPanel(); 
    drawMap();           
}

function updateMockData(id) {
    const c = allCartsData[id];
    if(!c.isOnline) return;
    if(Math.random() > 0.98) c.battery = Math.max(0, c.battery - 1);
    if(Math.random() > 0.7) c.speed = Math.floor(Math.random() * 80);
    // c.x += (Math.random() - 0.5) * 0.1;
    // c.y += (Math.random() - 0.5) * 0.1;
}

function renderGrid() {
    const grid = document.getElementById('cartGrid');
    grid.innerHTML = '';

    for(let i=1; i<=4; i++) {
        const data = allCartsData[i];
        const isSelected = (i === currentCartId) ? 'selected' : '';
        let dotClass = 'offline';
        let statusColor = '#64748b'; 

        if(data.isOnline) {
            if(data.status === 'WARNING') {
                dotClass = 'warning'; 
                statusColor = '#fbbf24'; 
            } else {
                dotClass = 'online';
                statusColor = '#34d399'; 
            }
        }
        
        const speedDisplay = data.isOnline ? `${data.speed} cm/s` : '--';

        const html = `
        <div class="cart-summary-card ${isSelected}" onclick="selectCart(${i})">
            <div class="summary-header">
                <div class="summary-name">${data.name}</div>
                <div class="summary-status-dot ${dotClass}" style="${dotClass === 'warning' ? 'background-color:#f97316; box-shadow:0 0 5px #f97316;' : ''}"></div>
            </div>
            <div class="summary-row" style="margin-top:5px;">
                <span>배터리</span>
                <span class="summary-val" style="color:${statusColor}">${data.isOnline ? data.battery+'%' : '-'}</span>
            </div>
            <div class="summary-row">
                <span>현재 속도</span> <span class="summary-val">${speedDisplay}</span> </div>
        </div>`;
        grid.innerHTML += html;
    }
}

function renderDetailPanel() {
    const data = allCartsData[currentCartId];
    const statusText = document.getElementById('detailStatusText');
    const panel = document.getElementById('detailPanel'); 

    document.getElementById('detailName').innerText = data.name;

    if(data.isOnline) {
        if(data.status === 'WARNING') {
            statusText.innerText = '● 무게 불일치 경고';
            statusText.style.color = '#fbbf24';
            panel.style.borderColor = '#fbbf24';
        } else {
            statusText.innerText = '● 온라인';
            statusText.style.color = '#34d399';
            panel.style.borderColor = '#334155'; 
        }
    } else {
        statusText.innerText = '● 오프라인';
        statusText.style.color = '#f87171';
        panel.style.borderColor = '#ef4444';
    }

    const elReal = document.getElementById('detailReal');
    const elExp = document.getElementById('detailExpected');
    const elBat = document.getElementById('detailBattery');
    const elSpd = document.getElementById('detailSpeed');

    if(data.isOnline) {
        elReal.innerText = typeof data.weight === 'number' ? data.weight.toFixed(1) : data.weight;
        elExp.innerText = typeof data.expected === 'number' ? data.expected.toFixed(1) : data.expected;
        elBat.innerText = data.battery;
        elSpd.innerText = data.speed;

        elBat.parentElement.className = data.battery < 20 ? "data-value val-warning" : "data-value val-success";
        if(data.status === 'WARNING') elReal.parentElement.className = "data-value val-warning";
        else elReal.parentElement.className = "data-value val-normal";
    } else {
        elReal.innerText = '--';
        elExp.innerText = '--';
        elBat.innerText = '--';
        elSpd.innerText = '--';
        elReal.parentElement.className = "data-value val-offline";
        elBat.parentElement.className = "data-value val-offline";
    }

    const tbody = document.getElementById('detailListBody');
    const countEl = document.getElementById('detailCount');
    const priceEl = document.getElementById('detailPrice');
    
    if(!data.isOnline || data.items.length === 0) {
        tbody.innerHTML = '<tr><td colspan="4" style="text-align: center; color: #64748b; padding: 30px;">데이터 없음</td></tr>';
        countEl.innerText = '0';
        priceEl.innerText = '0원';
    } else {
        const agg = {};
        let totalP = 0;
        let totalC = 0;
        data.items.forEach(item => {
            if(agg[item.name]) agg[item.name].qty++;
            else agg[item.name] = {...item, qty: 1};
            totalP += item.price;
            totalC++;
        });
        let html = '';
        Object.values(agg).forEach(item => {
            html += `<tr>
                <td style="font-weight:600;">${item.name}</td>
                <td style="text-align: right;">${item.price.toLocaleString()}원</td>
                <td style="text-align: right;">${item.weight}g</td>
                <td style="text-align: right; font-weight: bold; color: #38bdf8;">${item.qty}</td>
            </tr>`;
        });
        tbody.innerHTML = html;
        countEl.innerText = totalC;
        priceEl.innerText = totalP.toLocaleString() + '원';
    }
}

// [핵심] 지도 그리기 (로봇=빨강, 사람=파랑)
function drawMap() {
    const canvas = document.getElementById('robotMap');
    if(!canvas) return; 

    const ctx = canvas.getContext('2d');
    const data = allCartsData[currentCartId];

    // 1. 캔버스 리사이징 & 클리어
    const container = canvas.parentElement;
    if(canvas.width !== container.clientWidth || canvas.height !== container.clientHeight) {
        canvas.width = container.clientWidth;
        canvas.height = container.clientHeight;
    }
    const cvsW = canvas.width;
    const cvsH = canvas.height;
    ctx.clearRect(0, 0, cvsW, cvsH);

    if (mapImg.complete && mapImg.naturalWidth !== 0) {
        // 2. 맵 이미지 그리기 (Contain)
        const imgW = mapImg.naturalWidth;
        const imgH = mapImg.naturalHeight;
        const imgRatio = imgW / imgH;
        const cvsRatio = cvsW / cvsH;

        let drawW, drawH, drawX, drawY;

        if (cvsRatio > imgRatio) {
            drawH = cvsH;
            drawW = cvsH * imgRatio;
            drawX = (cvsW - drawW) / 2;
            drawY = 0;
        } else {
            drawW = cvsW;
            drawH = cvsW / imgRatio;
            drawX = 0;
            drawY = (cvsH - drawH) / 2;
        }
        ctx.drawImage(mapImg, drawX, drawY, drawW, drawH);
        
        // 3. 좌표 변환 함수
        const toPixel = (rosX, rosY) => {
            const xRange = MAP_BOUNDS.RIGHT_X - MAP_BOUNDS.LEFT_X; 
            let ratioX = (rosX - MAP_BOUNDS.LEFT_X) / xRange;
            
            const yRange = MAP_BOUNDS.BOTTOM_Y - MAP_BOUNDS.TOP_Y; 
            let ratioY = (rosY - MAP_BOUNDS.TOP_Y) / yRange;

            let finalX = drawX + (ratioX * drawW);
            let finalY = drawY + (ratioY * drawH);
            return { x: finalX, y: finalY };
        };

        if (data.isOnline) {
            // --- A. 로봇 그리기 (빨간색 점) ---
            const botPos = toPixel(data.x, data.y);
            
            ctx.beginPath();
            ctx.arc(botPos.x, botPos.y, 6, 0, Math.PI * 2);
            
            let color = (currentCartId === 1) ? '#ef4444' : (data.status === 'WARNING' ? '#fbbf24' : '#34d399');
            ctx.fillStyle = color;
            ctx.fill();
            
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2;
            ctx.stroke();

            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 12px sans-serif';
            ctx.fillText("BOT", botPos.x + 10, botPos.y + 4);

            // --- B. 사람 그리기 (파란색 점) - 1호차만 ---
            if (currentCartId === 1) {
                const humanPos = toPixel(data.human_x, data.human_y);

                ctx.beginPath();
                ctx.arc(humanPos.x, humanPos.y, 6, 0, Math.PI * 2);
                ctx.fillStyle = '#3b82f6'; // 파란색
                ctx.fill();
                
                ctx.strokeStyle = '#ffffff';
                ctx.stroke();

                ctx.fillStyle = '#3b82f6'; // 글자색도 파랑
                ctx.fillText("ME", humanPos.x + 10, humanPos.y + 4);
            }

            // 좌표 텍스트 (로봇 기준)
            const coordEl = document.getElementById('mapCoords');
            if(coordEl) coordEl.innerText = `Robot: (${data.x.toFixed(2)}, ${data.y.toFixed(2)})`;
        } else {
            const coordEl = document.getElementById('mapCoords');
            if(coordEl) coordEl.innerText = "OFFLINE";
        }
    } else {
        ctx.fillStyle = '#94a3b8';
        ctx.font = '14px sans-serif';
        ctx.fillText("Loading Map...", 20, 30);
    }
}

function selectCart(id) {
    currentCartId = id;
    renderGrid();       
    renderDetailPanel(); 
    drawMap();           
}

setInterval(updateDashboard, 1000);
updateDashboard();