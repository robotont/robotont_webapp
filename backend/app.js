var express = require('express');
var app = express();
var cors = require('cors');
var functions = require('./public/javascripts/functions.js');

app.use(cors());

app.post('/shutdown', function (req, res) {
    functions.shutdown();
    res.send('Success! Shutting down')
});

app.post('/rosRestart', function (req, res) {
    functions.rosRestart();
    res.send('Ros restart')
});

app.post('/rosStart', function (req, res) {
    functions.rosStart();
    res.send('Ros start')
});

app.post('/rosStop', function (req, res) {
    functions.rosStop();
    res.send('Ros stop')
})

app.listen(3000);
console.log("running at port 3000");