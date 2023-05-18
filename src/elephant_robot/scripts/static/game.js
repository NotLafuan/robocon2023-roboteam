function printMousePos(event) {
    document.body.textContent =
        "clientX: " + event.clientX +
        " - clientY: " + event.clientY;
    // $.post("/mouse", {
    //     x:event.clientX
    // });
    fetch('/mouse', {
        method: "POST",
        body: JSON.stringify({
            x: event.clientX,
            y: event.clientY
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
}

function detectKey(event) {
    if (event.keyCode != key_code){
        key_code = event.keyCode
        fetch('/key', {
            method: "POST",
            body: JSON.stringify({
                key: event.keyCode
            }),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
        })
            .then((response) => response.json())
            .then((json) => console.log(json))
    }
    
}

function detectRelease(event) {
    key_code = 0
    fetch('/key', {
        method: "POST",
        body: JSON.stringify({
            key: 0
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
}

key_code = 0
document.addEventListener("click", printMousePos);
document.addEventListener("keydown", detectKey);
document.addEventListener("keyup", detectRelease);
