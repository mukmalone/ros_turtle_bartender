//Author: Michael Muldoon
//email: michael.muldoon.home@gmail.com
//license: Apache 2.0
//Comment: This is a Node js Express webserver serving up random customers
// and drink orders for the turtle party of the year

const express = require('express')
const app = express()
const port = 3000

function getRandomInt(max) {
  return Math.floor(Math.random() * Math.floor(max));
}

app.get('/', (req, res) => {
  var numCustomers = req.query.num;
  var customerNum = getRandomInt(numCustomers);
  var customerName = 'Party_Turtle_' + customerNum.toString();

  var drinkNum = getRandomInt(4);
  var drinkName = '';
  switch(drinkNum) {
    case 0:
      drinkName='Red Wine';
      break;
    case 1:
      drinkName='White Wine';
      break;
    case 2:
      drinkName='White Wine';
      break;
    default:
      drinkName='Jamison';
      break;
  }
  
  console.log(customerName + "," + drinkName);

  res.send(customerName + "," + drinkName);
})

app.listen(port, () => {
  console.log(`Example app listening at http://localhost:${port}`)
})