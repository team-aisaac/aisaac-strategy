<template>
  <section class="section">
    <div id="wrapper" class="container">
      <article class="media">
        <div class="media-content">
          <div class="field is-grouped">
            <p class="control is-expanded">
              <input
                v-model="message"
                class="input"
                type="text"
                placeholder="message"
                @keyup.enter="sendMessage"
              >
            </p>
            <p class="control">
              <button class="button is-info" @click="sendMessage">
                Send
              </button>
            </p>
          </div>
        </div>
      </article>
      <article v-for="(message, index) in reverseMessages" :key="index" class="media">
        <figure class="media-left">
          <p class="image is-64x64">
            <img src="https://bulma.io/images/placeholders/128x128.png">
          </p>
        </figure>
        <div class="media-content">
          <div class="content">
            <p>
              <strong>id: {{ message.user }}</strong>
              <br>
              {{ message.text }}
              <br>
              <small>
                <a>Like</a> ·
                <a>Reply</a>
                · {{ message.date }}
              </small>
            </p>
          </div>
        </div>
      </article>
      <b-loading :is-full-page="false" :active.sync="isLoading" :can-cancel="false" />
    </div>
  </section>
</template>

<script>
import io from 'socket.io-client'

export default {
  data () {
    return {
      message: '',
      messages: [],
      socket: '',
      isLoading: true
    }
  },
  computed: {
    // 配列の後ろ（新しいもの）から順に表示させたいので反転させる
    reverseMessages () {
      return this.messages.slice().reverse()
    }
  },
  mounted () {
    // VueインスタンスがDOMにマウントされたらSocketインスタンスを生成する
    this.socket = io()

    // サーバー側で保持しているメッセージを受信する
    this.socket.on('new-message', (message) => {
      this.messages.push(message || {})
    })

    // コンポーネントがマウントされてから1秒間はローディングする
    setTimeout(() => {
      this.isLoading = false
    }, 1000)
  },
  methods: {
    sendMessage () {
      // スペースのみの場合は即時リターンをする
      if (!this.message.trim()) {
        return
      }

      let now = new Date() // 現在時刻（世界標準時）を取得
      now.setTime(now.getTime() + 1000 * 60 * 60 * 9) // 日本時間に変換
      now = now
        .toJSON()
        .split('T')[1]
        .slice(0, 5) // 時刻のみを取得

      // メッセージオブジェクトを作る
      const message = {
        user: this.socket.id,
        date: now,
        text: this.message.trim()
      }

      // 自身（Vueインスタンス）のデータオブジェクトにメッセージを追加する
      this.messages.push(message)
      // サーバー側にメッセージを送信する
      this.socket.emit('send-message', message)
      // input要素を空にする
      this.message = ''
    }
  }
}
</script>

<style scoped>
#wrapper {
  max-width: 600px;
}
</style>
