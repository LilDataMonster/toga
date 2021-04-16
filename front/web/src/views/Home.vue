<template>
  <v-container>
    <v-layout text-xs-center wrap>
      <v-flex xs12 sm6 offset-sm3>
        <v-card>
          <v-img :src="require('../assets/logo.png')" contain height="200"></v-img>
          <v-card-title primary-title>
            <div class="ma-auto">
              <span class="grey--text">IDF version: {{version}}</span>
              <br/>
              <span class="grey--text">ESP cores: {{cores}}</span>
              <br/>
              <v-card-text>
                <v-container fluid grid-list-lg>
                  <v-layout row wrap>
                    <v-flex xs8>
                      <v-slider v-model="wifi_duration" :min="10000" :max="9999999" label="WiFi Duration (ms)"></v-slider>
                    </v-flex>
                    <v-flex xs4>
                      <v-text-field v-model="wifi_duration" class="mt-0" type="number"></v-text-field>
                    </v-flex>
                    <v-flex xs8>
                      <v-slider v-model="ble_duration" :min="10000" :max="9999999" label="BLE Duration (ms)"></v-slider>
                    </v-flex>
                    <v-flex xs4>
                      <v-text-field v-model="ble_duration" class="mt-0" type="number"></v-text-field>
                    </v-flex>
                    <v-flex xs8>
                      <v-slider v-model="xbee_duration" :min="10000" :max="9999999" label="XBee Duration (ms)"></v-slider>
                    </v-flex>
                    <v-flex xs4>
                      <v-text-field v-model="xbee_duration" class="mt-0" type="number"></v-text-field>
                    </v-flex>
                    <v-flex>
                      <v-text-field v-model="post_url" label="Post URL" single-line></v-text-field>
                    </v-flex>
                  </v-layout>
                </v-container>
              </v-card-text>
            </div>
            <v-btn fab dark large color="purple darken-3" @click="start_board">
              <v-icon dark>check_box</v-icon>
            </v-btn>
          </v-card-title>
        </v-card>
      </v-flex>
    </v-layout>
  </v-container>
</template>

<script>
export default {
  data() {
    return {
      version: null,
      cores: null,
      wifi_duration: 120000,
      ble_duration: 120000,
      xbee_duration: 120000,
      post_url: "http://ldm-nodered.herokuapp.com/esp32"
    };
  },
  methods: {
    start_board: function() {
      this.$ajax
        .post("/config", {
          board: {
            operational: true,
            wifi_duration: this.wifi_duration,
            ble_duration: this.ble_duration,
            xbee_duration: this.xbee_duration,
            post_url: this.post_url
          }
        })
        .then(data => {
          console.log(data);
        })
        .catch(error => {
          console.log(error);
        });
    }
  },
  mounted() {
    // this.$ajax
    //   .get("/api/v1/system/info")
    //   .then(data => {
    //     this.version = data.data.version;
    //     this.cores = data.data.cores;
    //   })
    //   .catch(error => {
    //     console.log(error);
    //   });
  }
};
</script>
