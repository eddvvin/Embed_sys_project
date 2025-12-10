// Import Firebase CDN's
import { initializeApp } from "https://www.gstatic.com/firebasejs/12.6.0/firebase-app.js";
import { getDatabase, ref, onValue, set } from "https://www.gstatic.com/firebasejs/12.6.0/firebase-database.js";

// Firebase configuration
const firebaseConfig = {
    apiKey: "AIzaSyBayJmG5l9niRQW67GumaUSABTCMyhmato",
    authDomain: "embed-sys-project.firebaseapp.com",
    databaseURL: "https://embed-sys-project-default-rtdb.firebaseio.com",
    projectId: "embed-sys-project",
    storageBucket: "embed-sys-project.firebasestorage.app",
    messagingSenderId: "341521528749",
    appId: "1:341521528749:web:b8cb737c2dfce38ea75324",
    measurementId: "G-KH6MN65XQW"
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const db = getDatabase(app);

//Config to allow for scalability with relays
const relays = [
    {
        name: 'Relay 1',
        dataPath: 'RelayData1', //Firebase path
        toggleId: 'relayToggle1', //ID for switch
        chartId: 'currentChart1',
        color: 'blue',
        datePickerId: 'dateRelay1'
    },
    {
        name: 'Relay 2',
        dataPath: 'RelayData2',
        toggleId: 'relayToggle2',
        chartId: 'currentChart2',
        color: 'red',
        datePickerId: 'dateRelay2'
    }
];

function filterDataByDate(data, selectedDate) {
    if (!data || !selectedDate) return data;

    const filtered = {};
    const [year, month, day] = selectedDate.split('-').map(Number); // yyyy-mm-dd

    Object.entries(data).forEach(([ts, value]) => {
        const d = new Date(Number(ts));
        if (d.getFullYear() === year &&
            d.getMonth() + 1 === month &&
            d.getDate() === day) {
            filtered[ts] = value;
        }
    });

    return filtered;
}

//Create a chart and it's updating configuration for each relay
relays.forEach(relay => { //Dynamically creates charts

    const ctx = document.getElementById(relay.chartId);
    relay.chart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [], //x-axis
            datasets: [{
                label: `Current`,
                data: [], //y-axis
                borderColor: relay.color,
                tension: 0.2
            }]
        }
    });

// Firebase references
relay.dataRef = ref(db, relay.dataPath);
relay.toggleRef = ref(db, relay.toggleId);
relay.datePicker = document.getElementById(relay.datePickerId);
relay.toggle = document.getElementById(relay.toggleId);

// Function to update chart
relay.updateChart = (data) => {
    data = filterDataByDate(data, relay.datePicker.value);
    relay.chart.data.labels = Object.keys(data).map(ts => new Date(Number(ts)).toLocaleTimeString()); //Converts timestamp to date and time
    relay.chart.data.datasets[0].data = Object.values(data);
    relay.chart.update();
};

// Listen for data changes
onValue(relay.dataRef, (snapshot) => {
    const data = snapshot.val();
    relay.updateChart(data);
});

// Update chart when date picker changes
relay.datePicker.addEventListener('change', () => {
    onValue(relay.dataRef, (snapshot) => {
        const data = snapshot.val();
        relay.updateChart(data);
    }, { onlyOnce: true });
});

// Keep toggle synced
onValue(relay.toggleRef, (snapshot) => {
    const val = snapshot.val();
    relay.toggle.checked = val === 1;
});

// Update toggle in Firebase
relay.toggle.addEventListener('change', () => {
    const newVal = relay.toggle.checked ? 1 : 0;
    set(relay.toggleRef, newVal);
});
});
