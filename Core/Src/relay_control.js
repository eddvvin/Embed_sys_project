import { initializeApp } from "https://www.gstatic.com/firebasejs/10.7.1/firebase-app.js";
import { getDatabase, ref, set, onValue } from "https://www.gstatic.com/firebasejs/10.7.1/firebase-database.js";

const firebaseConfig = {
    apiKey: "AIzaSyBayJmG5l9niRQW67GumaUSABTCMyhmato",
    authDomain: "embed-sys-project.firebaseapp.com",
    databaseURL: "https://embed-sys-project-default-rtdb.firebaseio.com/",
    projectId: "embed-sys-project",
    storageBucket: "embed-sys-project.appspot.com",
    messagingSenderId: "XXXXXXXXXXXX",
    appId: "1:XXXXXXXXXXXX:web:XXXXXXXXXXXX"
};

const app = initializeApp(firebaseConfig);
const db = getDatabase(app);


const relay1 = document.getElementById("relayToggle1");
const relay2 = document.getElementById("relayToggle2");

onValue(ref(db, "relayToggle1"), (snapshot) => {
    relay1.checked = snapshot.val() === 1;
});

onValue(ref(db, "relayToggle2"), (snapshot) => {
    relay2.checked = snapshot.val() === 1;
});
relay1.addEventListener("change", () => {
    set(ref(db, "relayToggle1"), relay1.checked ? 1 : 0);
});

relay2.addEventListener("change", () => {
    set(ref(db, "relayToggle2"), relay2.checked ? 1 : 0);
});
