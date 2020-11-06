const express = require('express')
const app = express()
const port = 3000

app.get('/', (req, res) => {
  res.send('Party_Turtle_1,Red Wine')
})

app.listen(port, () => {
  console.log(`Example app listening at http://localhost:${port}`)
})