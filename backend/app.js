const express = require('express');
const app = express();
const cors = require('cors');
const { exec, spawn } = require('child_process');
//const functions = require('./public/javascripts/functions.js');
const sh = spawn('bash');

app.use(cors());

app.post('/shutdown', function (req, res) {
    exec("sudo poweroff", (error, stdout, stdrr) => {
        if (error) {
            console.error(`error: ${error.message}`);
        }

        if (stdrr) {
            console.error(`stderr: ${stdrr}`)
        }

        console.log(`stdout:\n${stdout}`);
    });
    res.send('Success! Shutting down')
});

//'sudo systemctl restart robotont.service'
app.post('/rosRestart', function (req, res) {
    exec("sudo systemctl restart robotont.service", (error, stdout, stdrr) => {
        if (error) {
            console.error(`error: ${error.message}`);
        }

        if (stdrr) {
            console.error(`stderr: ${stdrr}`)
        }

        console.log(`stdout:\n${stdout}`);
    });
    res.send('Ros restart')
});

app.post('/rosStart', function (req, res) {
    exec("sudo systemctl start robotont.service", (error, stdout, stdrr) => {
        if (error) {
            console.error(`error: ${error.message}`);
        }

        if (stdrr) {
            console.error(`stderr: ${stdrr}`)
        }

        console.log(`stdout:\n${stdout}`);
    });
    res.send('Ros start')
});

app.post('/rosStop', function (req, res) {
    exec("sudo systemctl stop robotont.service", (error, stdout, stdrr) => {
        if (error) {
            console.error(`error: ${error.message}`);
        }

        if (stdrr) {
            console.error(`stderr: ${stdrr}`)
        }

        console.log(`stdout:\n${stdout}`);
    });
    res.send('Ros stop')
});

app.listen(3000, function() {
    console.log("running at port 3000");
});

