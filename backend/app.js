var express = require('express');
var app = express();
var cors = require('cors');
var functions = require('./public/javascripts/test.js');

app.use(cors());

app.get('/shutdown', function (req, res) {
    functions.shutdown();
    res.send('Success! Shutting down')
});

app.get('/rosRestart', function (req, res) {
    functions.rosRestart();
    res.send('Ros restart')
});

app.get('/rosStart', function (req, res) {
    functions.rosStart();
    res.send('Ros start')
});

app.get('/rosStop', function (req, res) {
    functions.rosStop();
    res.send('Ros stop')
})

app.listen(3000);
console.log("running at port 3000");