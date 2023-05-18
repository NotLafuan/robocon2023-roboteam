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

function detectRelease(event) {
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

document.addEventListener("click", printMousePos);
document.addEventListener("keypress", detectKey);
document.addEventListener("keyup", detectRelease);
