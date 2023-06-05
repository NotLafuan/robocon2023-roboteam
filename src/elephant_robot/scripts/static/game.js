function detectMousePos(event) {
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
    if (event.keyCode != key_code) {
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
document.addEventListener("click", detectMousePos);
document.addEventListener("keydown", detectKey);
document.addEventListener("keyup", detectRelease);

// Add event listeners to the buttons
document.getElementById("up").addEventListener("mousedown", function () {
    console.log("Up button clicked");
    fetch('/button', {
        method: "POST",
        body: JSON.stringify({
            button: "up"
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
});

document.getElementById("up").addEventListener("mouseup", function () {
    fetch('/button', {
        method: "POST",
        body: JSON.stringify({
            button: 0
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
});

document.getElementById("left").addEventListener("mousedown", function () {
    console.log("Left button clicked");
    fetch('/button', {
        method: "POST",
        body: JSON.stringify({
            button: "left"
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
});

document.getElementById("left").addEventListener("mouseup", function () {
    fetch('/button', {
        method: "POST",
        body: JSON.stringify({
            button: 0
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
});

document.getElementById("right").addEventListener("mousedown", function () {
    console.log("Right button clicked");
    fetch('/button', {
        method: "POST",
        body: JSON.stringify({
            button: "right"
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
});

document.getElementById("right").addEventListener("mouseup", function () {
    fetch('/button', {
        method: "POST",
        body: JSON.stringify({
            button: 0
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
});

document.getElementById("down").addEventListener("mousedown", function () {
    console.log("Down button clicked");
    fetch('/button', {
        method: "POST",
        body: JSON.stringify({
            button: "down"
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
});
document.getElementById("down").addEventListener("mouseup", function () {
    fetch('/button', {
        method: "POST",
        body: JSON.stringify({
            button: 0
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
        .then((response) => response.json())
        .then((json) => console.log(json))
});
