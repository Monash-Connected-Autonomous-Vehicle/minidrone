$(document).ready(function () {
  const ctx = document.getElementById("myChart").getContext("2d");

  const myChart = new Chart(ctx, {
    type: "line",
    data: {
      datasets: [
        { label: "Fan speed",
          borderColor: "#FF5733",
          showLine: true,
          
        }, 
        { label: "Temp",
          borderColor: "#3374FF",
          showLine: true,
        }
      ],
    },
    options: {
      borderWidth: 1,
      elements: {
        point: {
          radius: 0
        }
      },
      scales: {
        y: {
          title: {
            display: true,
            text: "Value"
          }
        },
        x: {
          title: {
            display: true,
            text: "Time"
          }
        }
      }
      
    },
  });

  function addData(label, data) {
    myChart.data.labels.push(label);
    myChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myChart.update();

    console.log("CURRENT DATA:", myChart.data.datasets[0].data)
  }

  function removeFirstData() {
    myChart.data.labels.splice(0, 1);
    myChart.data.datasets.forEach((dataset) => {
      dataset.data.shift();
    });
  }

  const MAX_DATA_COUNT = 50;
  //connect to the socket server.
  //   var socket = io.connect("http://" + document.domain + ":" + location.port);
  var socket = io.connect();

  console.log("test")

  //receive details from server
  socket.on("QDES", function (msg) {
    console.log("Received sensorData :: " + msg.fan_speed + " :: " + msg.temp);
    console.log("Timestep: ", msg.time)

    // Show only MAX_DATA_COUNT data
    if (myChart.data.labels.length > MAX_DATA_COUNT) {
      removeFirstData();
    }
    // addData(msg.time, msg.fan_speed);
    // addData(msg.time, msg.temp);
    myChart.data.labels.push(msg.time);
    // myChart.data.datasets.forEach((dataset) => {
    //   dataset.data.push(data);
    // });
    myChart.data.datasets[0].data.push(msg.fan_speed)
    myChart.data.datasets[1].data.push(msg.temp)
    myChart.update();
  });
});
