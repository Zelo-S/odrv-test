<script>
	let text_now = "";
	const press = () => {
		fetch("/press_button")
		.then(d => d.text())
		.then(d => {text_now = d})
	}
	
	let speed = 0.0;
	let pos_gain = 0.0;
	let vel_gain = 0.0;
	let vel_integrator_gain = 0.0;
	
	const notValidSpeed = (requested_speed) => {
		if (requested_speed < 0 || requested_speed > 1){
			return true;
		}
		return false;
	}

	const postSpeed = () => {
		let toyData = {
			speed_: speed,
		}	
		fetch("/post/speed", {
			headers: {
				"Content-Type": "application/json",
			}, 
			method: "POST", 
			body: JSON.stringify(toyData)
		});
	}
	
	const postCalibrationSignal = () => {
		fetch("/post/calibration", {
			method: "POST", 
		});
	}

	const postZeroingSignal = () => {
		fetch("/post/zero", {
			method: "POST", 
		});
	}
	const postIdleSignal = () => {
		fetch("/post/idle", {
			method: "POST", 
		});
	}
	const postCLCSignal = () => {
		fetch("/post/clc", {
			method: "POST", 
		});
	}
	
	const postHomeSignal = () => {
		fetch("/post/home", {
			method: "POST", 
		});
	}

	const postPosGainSignal = () => {
		let toyData = {
			pos_gain: pos_gain,
		}	
		fetch("/post/pos_gain", {
			headers: {
				"Content-Type": "application/json",
			}, 
			method: "POST", 
			body: JSON.stringify(toyData)
		});
	}

	const postVelGainSignal = () => {
		let toyData = {
			vel_gain: vel_gain,
			vel_integrator_gain: vel_integrator_gain
		}	
		fetch("/post/vel_gain", {
			headers: {
				"Content-Type": "application/json",
			}, 
			method: "POST", 
			body: JSON.stringify(toyData)
		});
	}

	let set_pos = 0.0;
	const postSetPosSignal = () => {
		let toyData = {
			set_pos: set_pos,
		}	
		fetch("/post/set_pos", {
			headers: {
				"Content-Type": "application/json",
			}, 
			method: "POST", 
			body: JSON.stringify(toyData)
		});
	}

	let set_vel = 0.0;
	const postSetVelSignal = () => {
		let toyData = {
			set_vel: set_vel,
		}	
		fetch("/post/set_vel", {
			headers: {
				"Content-Type": "application/json",
			}, 
			method: "POST", 
			body: JSON.stringify(toyData)
		});
	}
	
	const postCANSetupSignal = () => {
		let toyData = {
			
		}
	}

</script>

<main>
	<button class="stop" on:click={postIdleSignal}>STOP</button>
	<br>
	
	<p>CAN Toggle</p>
	<input bind:value={pos_gain} placeholder="enter pos_gain(0-1)"/>
	<button id="toggleButton" on:click={postPosGainSignal}>Send pos_gain:{pos_gain}</button>

	<p>Set Pos Gain</p>
	<input bind:value={pos_gain} placeholder="enter pos_gain(0-1)"/>
	<button on:click={postPosGainSignal}>Send pos_gain:{pos_gain}</button>

	<br>
	<p>Set Vel Gains</p>
	<input bind:value={vel_gain} placeholder="enter vel_gain(0-1)"/>
	<input bind:value={vel_integrator_gain} placeholder="enter vel_integrator_gain(0-1)"/>
	<button on:click={postVelGainSignal}>Send vel_gain:{vel_gain}, integrator_gain:{vel_integrator_gain}</button>

	<p>Set Pos</p>
	<input bind:value={set_pos} placeholder="enter pos)"/>
	<button on:click={postSetPosSignal}>Send setpos:{set_pos}</button>

	<!-- <p>Set Vel</p>
	<input bind:value={set_vel} placeholder="enter vel)"/>
	<button on:click={postSetVelSignal}>Send setvel:{set_vel}</button>-->

	<br>
	<br>
	<button on:click={postCLCSignal}>CLC</button>
	<br>
	<button on:click={postCalibrationSignal}>Calibrate</button>
	<button on:click={postZeroingSignal}>Set Linear Count 0 + Tune PID</button>
	<br>
	<button on:click={postHomeSignal}>Home to 0</button>
	<br>
	
</main>

<style>
	button {
		margin: 3px;
	}
	
	.stop {
		display: block;
		width: 100%;
	}
</style>
