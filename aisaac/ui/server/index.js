'use strict'

const express = require('express')
const consola = require('consola')
const {
  Nuxt,
  Builder
} = require('nuxt')
const app = express()

// Import ROS Libraries
const rosnodejs = require('rosnodejs')
const stdMsgs = rosnodejs.require('std_msgs').msg
let pub = null

// Import and Set Nuxt.js options
const config = require('../nuxt.config.js')
config.dev = process.env.NODE_ENV !== 'production'

async function start () {
  // Init Nuxt.js
  const nuxt = new Nuxt(config)

  const {
    host,
    port
  } = nuxt.options.server

  // Build only in dev mode
  if (config.dev) {
    const builder = new Builder(nuxt)
    await builder.build()
  } else {
    await nuxt.ready()
  }

  // Give nuxt middleware to express
  app.use(nuxt.render)

  // Listen the server
  const server = app.listen(port, host)
  consola.ready({
    message: `Server listening on http://${host}:${port}`,
    badge: true
  })
  // Listen the socket server
  socketStart(server)
  consola.info('Socket.IO starts')
}

function rosSend (message) {
  const msg = new stdMsgs.String()
  msg.data = 'Chat: ' + message.text
  pub.publish(msg)
}

let messageQueue = []

function socketStart (server) {
  // Websocketサーバーインスタンスを生成する
  const io = require('socket.io').listen(server)
  // クライアントからサーバーに接続があった場合のイベントを作成する
  io.on('connection', (socket) => {
    // Initialize ros node
    rosnodejs.initNode('/talker_node').then((nh) => {
      pub = nh.advertise('/chatter', stdMsgs.String)
      nh.subscribe('/yellow/referee/referee_branch', stdMsgs.String,
        (data) => {
          socket.emit('hello')
        })
    })

    consola.info('ROS Node starts')

    // 接続されたクライアントのidをコンソールに表示する
    consola.info('id: ' + socket.id + ' is connected')
    // サーバー側で保持しているメッセージをクライアント側に送信する
    if (messageQueue.length > 0) {
      messageQueue.forEach((message) => {
        socket.emit('new-message', message)
      })
    }
    // クライアントから送信があった場合のイベントを作成する
    socket.on('send-message', (message) => {
      consola.info(message)
      // サーバーで保持している変数にメッセージを格納する
      messageQueue.push(message)
      // 送信を行ったクライアント以外のクライアントに対してメッセージを送信する
      socket.broadcast.emit('new-message', message)
      // ROSトピック送信
      rosSend(message)
      // サーバー側で保持しているメッセージが10を超えたら古いものから削除する
      if (messageQueue.length > 10) {
        messageQueue = messageQueue.slice(-10)
      }
    })
  })
}

start()
