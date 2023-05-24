$(document).ready(function () {
    var socket = io.connect();
    // var rectangleSet = false;
    
    var canvas1 = document.getElementById('chart1').getContext('2d');
    var canvas2 = document.getElementById('chart2').getContext('2d'); 
    var canvas3 = document.getElementById('chart3').getContext('2d'); 
    var canvas4 = document.getElementById('chart4').getContext('2d'); 
    // var canvas5 = document.getElementById('chart5').getContext('2d'); 


    // const data = {
    //     datasets: [
    //         {
    //             label: 'Fan Speed',
    //             // backgroundColor: Utils.transparentize(Utils.CHART_COLORS.red, 0.5),
    //             borderColor:"#E84524",
    //             borderDash: [8, 4],
    //             data: []
    //         },
    //     ]
    // };

    const data = {
        datasets: [
            {
                label: 'Temperature (Veroboard)',
                // backgroundColor: Utils.transparentize(Utils.CHART_COLORS.red, 0.5),
                borderColor: "#E84524",
                borderDash: [8, 4],
                data: []
            },
        ]
    };


    const data2 = {
        datasets: [
            {
                label: 'Temperature (Intake)',
                // backgroundColor: Utils.transparentize(Utils.CHART_COLORS.red, 0.5),
                borderColor: "#3374FF",
                borderDash: [8, 4],
                data: []
            },
        ]
    };


    const data3 = {
        datasets: [
            {
                label: 'Temperature (Exhaust)',
                // backgroundColor: Utils.transparentize(Utils.CHART_COLORS.red, 0.5),
                borderColor: "#68E824",
                borderDash: [8, 4],
                data: []
            },
        ]
    };

    const data4 = {
        datasets: [
            {
                label: 'TX2 Temperature',
                // backgroundColor: Utils.transparentize(Utils.CHART_COLORS.red, 0.5),
                borderColor: "#E8AD24",
                borderDash: [8, 4],
                data: []
            },
        ]
    };

    const config = {
        type: 'line',
        data: data,
        options: {
            maintainAspectRatio: true,
            scales: {
                x: {
                    type: 'realtime',
                    realtime: {
                        // duration: 20000,
                        refresh: 1000,
                        delay: 500,
                        ttl: 30000,
                        onRefresh: function (chart) {
                            onRefresh(chart, 0)
                        } 
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Value'
                    }
                }
            },
            interaction: {
                intersect: false
            }
        }
    };

    const config2 = {
        type: 'line',
        data: data2,
        options: {
            maintainAspectRatio: true,
            scales: {
                x: {
                    type: 'realtime',
                    realtime: {
                        // duration: 20000,
                        refresh:1000,
                        delay: 500,
                        ttl: 30000,
                        // onRefresh: function (chart) {
                        //     onRefresh(chart, 1)
                        // } 
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Value'
                    }
                }
            },
            interaction: {
                intersect: false
            }
        }
    };

    const config3 = {
        type: 'line',
        data: data3,
        options: {
            maintainAspectRatio: true,
            scales: {
                x: {
                    type: 'realtime',
                    realtime: {
                        // duration: 20000,
                        refresh:1000,
                        delay: 500,
                        ttl: 30000,
                        // onRefresh: function (chart) {
                        //     onRefresh(chart, 2)
                        // } 
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Value'
                    }
                }
            },
            interaction: {
                intersect: false
            }
        }
    };


    const config4 = {
        type: 'line',
        data: data4,
        options: {
            maintainAspectRatio: true,
            scales: {
                x: {
                    type: 'realtime',
                    realtime: {
                        // duration: 20000,
                        refresh:1000,
                        delay: 500,
                        ttl: 30000,
                        // onRefresh: function (chart) {
                        //     onRefresh(chart, 2)
                        // } 
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Value'
                    }
                }
            },
            interaction: {
                intersect: false
            }
        }
    };


    var chart1 = new Chart(canvas1, config);
    var chart2 = new Chart(canvas2, config2);
    var chart3 = new Chart(canvas3, config3);
    var chart4 = new Chart(canvas4, config4);


    const onRefresh = (chart, chartIdx) => {
        socket.on('QDES', function (msg) {
            const now = Date.now();
            const latestDataTimestamp = chart.data.datasets[0].data[chart.data.datasets[0].data.length - 1]?.x;

            console.log(latestDataTimestamp, now)

            var y =0;

            if (latestDataTimestamp !== now) {
                // console.log("adding for: ", latestDataTimestamp)
                if (latestDataTimestamp !== now) {
                    console.log(msg);

                    // switch (chartIdx) {
                    // case 0:
                    //     y = msg.fan_speed;
                    //     break;
                    // case 1:
                    //     y = msg.temp;
                    //     break;
                    // case 2:
                    //     y = msg.tx2_temp;
                    //     break;
                    // default:
                    //     break;
                    // }

                    chart.data.datasets[0].data.push({
                        x: now,
                        y: msg.temp1,
                    });
                    chart2.data.datasets[0].data.push({
                        x: now,
                        y: msg.temp2
                    })
                    chart3.data.datasets[0].data.push({
                        x: now,
                        y: msg.temp3
                    })
                    chart4.data.datasets[0].data.push({
                        x: now,
                        y: msg.TX2_temp,
                    })
                }
            }
        });
    }
    
})