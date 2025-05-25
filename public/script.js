function verde_rosu() {
  const pElement = document.getElementById("puls");
  const sElement = document.getElementById("saturatie");

  const p = parseInt(pElement.textContent);
  const s = parseInt(sElement.textContent);

  if (!isNaN(p)) {
   if (p > 100 || p < 50) {
      pElement.style.color = "red";
    } else {
      pElement.style.color = "green";
    }
  }

  if (!isNaN(s)) {
    sElement.style.color = s < 90 ? "red" : "green";
  }
}

document.addEventListener("DOMContentLoaded", () => {
  verde_rosu();
});

let intervalId;

function start() {
  intervalId = setInterval(() => {
    const puls = Math.floor(Math.random() * 60) + 60; 
    const saturatie = Math.floor(Math.random() * 15) + 85; 

    document.getElementById("puls").textContent = puls;
    document.getElementById("saturatie").textContent = saturatie;

    verde_rosu();
  }, 1000);
}

function stop() {
  clearInterval(intervalId);
}
