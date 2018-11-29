(
s.options.numOutputBusChannels_(4);
s.options.memSize_(8192 * 8);
s.waitForBoot({
	// Parameters
	var sndPath, oscDefKey, soundInPort, recordingDur, jerkOSCKey, positionOSCKey;
	var sectionOneDur, sectionTwoDur, sectionThreeDur, pulse, pulseRate, startTime;
	// Kernels
	var diffuseEncoder, omniEncoder, binauralDecoder;
	var spreadEncoder, stereoDecoder;
	// Buffers
	var sounds, voiceRecBuf;
	// Busses
	var voiceSpatializerBus, voicePositionBus, decoderBus;
	var sidechainBus, sidechainPulseBus, recPosBus, recBus;
	// Groups
	var positionGroup, synthesisGroup, spatializerGroup, sidechainGroup;
	var decoderGroup;
	// Helper Functions
	var calcFeedbackFromDecayTime;
	// Synth defs
	var pulseSynth, decoderSynth, spatializerSynth, positionSynth;
	var breathSynth, droneSynth, sidechainSynth, breathPercSynth;
	var recSynth, recPosSynth;
	// Routines / sections
	var scoreRoutine, persistentRoutine, pulseBreathRoutine, shepherdSwarmRoutine;
	var breathPercRoutine;
	// GUI
	var window, userView;

	// Parameters -------------------------------------
	sndPath = "../res/".resolveRelative;
	oscDefKey = \synthesisOSC;
	jerkOSCKey = \jerkTriggerOSC;
	positionOSCKey = \positionOSC;
	soundInPort = 0;
	recordingDur = 60 * 60;
	pulse = "Pulse2";
	pulseRate = 1.0;

	// Kernels ----------------------------------------
	diffuseEncoder = FoaEncoderKernel.newDiffuse;
	omniEncoder = FoaEncoderMatrix.newOmni;
	spreadEncoder = FoaEncoderKernel.newSpread;
	binauralDecoder = FoaDecoderKernel.newListen;
	stereoDecoder = FoaDecoderMatrix.newStereo;
	s.sync;

	// Buffers ----------------------------------------
	sounds = Dictionary.new;
	(sndPath ++ "*").pathMatch.collect{
		arg bufferPath;
		var buffer;
		buffer = CtkBuffer.playbuf(bufferPath).load;
		sounds.put(bufferPath.basename.splitext[0], buffer);
	};

	voiceRecBuf = CtkBuffer.buffer(recordingDur * s.sampleRate, 1, s).load;

	// Busses -----------------------------------------
	voiceSpatializerBus = CtkAudio.new(4);
	voicePositionBus = CtkControl.new(3);
	decoderBus = CtkAudio.new(4);
	sidechainBus = CtkAudio.new(4);
	sidechainPulseBus = CtkAudio.new(1);
	recBus = CtkAudio.new(1);
	recPosBus = CtkAudio.new(1);

	// Groups -----------------------------------------
	positionGroup = CtkGroup.play(server: s);
	s.sync;
	synthesisGroup = CtkGroup.play(addAction: 'after', target: positionGroup, server: s);
	s.sync;
	spatializerGroup = CtkGroup.play(addAction: 'after', target: synthesisGroup, server: s);
	s.sync;
	sidechainGroup = CtkGroup.play(addAction: 'after', target: spatializerGroup, server: s);
	s.sync;
	decoderGroup = CtkGroup.play(addAction: 'after', target: sidechainGroup, server: s);
	s.sync;

	// Helper Functions -------------------------------
	calcFeedbackFromDecayTime = {
		// expects positive non-zero values.
		arg decayTime, delay, target = -60.dbamp;

		target.pow(delay / decayTime);
	};


	// Synth Defs -------------------------------------
	pulseSynth = CtkSynthDef.new(\pulseSynth, {
		arg gain, dur = 1.0, rate, pulseBuffer, outBus, sidechainOutBus;

		var numBufChannels;
		var out, env;

		numBufChannels = 1;

		out = PlayBuf.ar(numBufChannels, pulseBuffer, rate, loop: 1);
		out = FoaEncode.ar(out, diffuseEncoder);

		env = EnvGen.ar(Env([1, 1, 0], [dur, 0.1], 0), doneAction: 2);

		out = out * env;


		Out.ar(outBus, out * gain.dbamp);
		Out.ar(sidechainOutBus, out[0]);
	});

	breathSynth = CtkSynthDef.new(\breathSynth, {
		arg gain = 0.0, dur = 1.0, delay = 0.1, maxDelay = 2.0, t60 = 10, tOct = 5.0,
		pitchUp = 1.0, buffer, outBus, recBus, pos;

		var maxReadDelay, readDelay, grainFreq, rate;
		var signal, impulse, gdur, ratePos, out;
		var localIn, localChannels;
		var feedback, rateTarget;
		var env, recOut;

		// constant parameters
		maxReadDelay = 2;
		readDelay = ControlDur.ir;
		grainFreq = 100;
		rate = 1.0;
		localChannels = 1;

		// lag user input
		gain = Lag2.kr(gain);
		delay = Lag2.kr(delay);
		t60 = Lag2.kr(t60);
		tOct = Lag2.kr(tOct);

		// input based parameters
		feedback = calcFeedbackFromDecayTime.value(t60, delay, -60.dbamp);
		rateTarget = (2.0 * pitchUp) + ((1 / 2.0) * (1 - pitchUp));
		rate = calcFeedbackFromDecayTime.value(tOct, delay, rateTarget);

		// Get the microphone input
		signal = SoundIn.ar(soundInPort);
		//signal = PinkNoise.ar(0.5) * EnvGen.ar(Env([1, 1, 0, 0], [0, 1, 0], -10, 2, 0));
		//signal = BPF.ar(PinkNoise.ar(0.5), 880, 1);
		//signal = SinOsc.ar(200, 0, 0.5);

		// record the microphone input and combine with the feedback
		localIn = LocalIn.ar(localChannels);
		localIn = DelayC.ar(localIn, maxDelay, delay);

		recOut = signal + (feedback * localIn);

		// granular synthesis calculations
		impulse = Impulse.ar(grainFreq);
		gdur = 2 / grainFreq;
		ratePos = ratePos = (pos - (max(0, rate - 1) * gdur * SampleRate.ir / BufSamples.ir(buffer))) % 1;

		// granular synthesis
		out = GrainBuf.ar(1, impulse, gdur, buffer, rate, ratePos);

		env = EnvGen.ar(Env([1, 1, 0], [dur, 0.1], 0), doneAction: 2);

		out = out * env;

		// feedback out (pre-encoding)
		LocalOut.ar(out);

		// Ambisonic encode
		out = FoaEncode.ar(out, diffuseEncoder);

		Out.ar(outBus, out);
		Out.ar(recBus, recOut);
	});

	breathPercSynth = CtkSynthDef.new(\breathPercSynth, {
		arg gain;
		Out.ar(0, SinOsc.ar);
	});

	droneSynth = CtkSynthDef.new(\droneSynth, {
		arg gain, dur = 1.0, gFreq, rate, driftRate, index, buffer, outBus;

		var out, gdur, impulse, drift, env;

		gdur = 2 / gFreq;
		impulse = Impulse.ar(gFreq);
		drift = LFNoise2.ar(driftRate, pi / 2, [0, 0]);

		out = GrainBuf.ar(1, impulse, gdur, buffer, rate, index);
		out = FoaPanB.ar(out, drift[0], drift[1]);
		out = out * gain.dbamp;

		env = EnvGen.ar(Env([1, 1, 0], [dur, 0.1], 0), doneAction: 2);
		out = out * env;

		Out.ar(outBus, out);
	});

	sidechainSynth = CtkSynthDef.new(\sidechainSynth, {
		arg gain, inBus, pulseBus, sidechainGain, depth, dur, rate, outBus;

		var env, out, pulse, max, lag, scale, range;

		max = 0.5;
		lag = 0.05;
		range = 1 - depth;
		range = 1 - depth;
		scale = max / range;

		out = In.ar(inBus, 4);
		pulse = In.ar(pulseBus, 1);

		// sidechain dependent on pulse
		out = out * (1 - min(1, Lag2.ar(pulse.abs) * sidechainGain.dbamp));
		out = out * gain.dbamp;

		Out.ar(outBus, out);
	});



	decoderSynth = CtkSynthDef.new(\decoderSynth, {
		arg inBus;

		var outBus, out;

		outBus = 0;

		out = In.ar(inBus, 4);
		out = FoaDecode.ar(out, binauralDecoder);

		Out.ar(outBus, out);
	});

	recSynth = CtkSynthDef.new(\recSynth, {
		arg buffer, inBus;

		var in, numChannels;

		numChannels = 1;

		in = In.ar(inBus, numChannels);

		RecordBuf.ar(in, buffer);
	});

	recPosSynth = CtkSynthDef.new(\recPosSynth, {
		arg delay, bufDur, recPosOut;

		var pos;

		pos = DelayN.ar(Line.ar(0, 1, bufDur), delay, delay);

		Out.ar(recPosOut, pos);
	});

	positionSynth = CtkSynthDef.new(\positionSynth, {
		arg inX = 0.5, inY = 0.5, inZ = 0.5,
		inXVel = 0, cubeRoomSideLen, outBus;

		var outX, outY, outZ, jerk, lastJerk;

		jerk = inXVel.abs > 1500;

		Poll.kr(jerk, inXVel, 'j');

		SendTrig.kr(jerk);

		#inX, inY, inZ = Lag2.kr([inX, inY, inZ], 0.05);

		// transform from [0, 1] to [-1, 1]
		#outX, outY, outZ = ([inX, inY, inZ] - 0.5) / 0.5;

		// flip the x-axis
		outX = outX * -1;

		// Scale to room size
		#outX, outY, outZ = [outX, outY, outZ] * cubeRoomSideLen;

		// translate between Leap coordinate axes to
		// traditional audio / ambisonic axes.
		Out.kr(outBus, [outZ, outX, outY]);
	});

	// Dan's spatializer
	spatializerSynth = CtkSynthDef.new(\spatializerSynth, {
		arg inbus, positionbus, outbus, width, gain = -12,
		maxMeters = 1000, reverbGain = -18;

		var pos, cartesian, src, amplitude, freq, radialDistance, amp;
		var phi, theta,  chain, hiFreqAttenCoeff, minAttenDist, lpfDB, speedOfSound;
		var reverb;

		// lets set a few variables first
		// our high frequency absorption constant
		hiFreqAttenCoeff = 100000;

		// at 15 meters our LPF freq would be 20,000, there's no point in
		// having a LPF higher than that
		minAttenDist = 15;

		// our LPF's cutoff freq is always at -3dB, so this is what freq we
		// need to calculate
		lpfDB = 3.0;

		// the speed of sound at sea level under normal conditions
		speedOfSound = 344;

		//convert gain to amplitude scalar
		amp = gain.dbamp;

		// B Format input
		src = In.ar(inbus, 4);

		// Position input
		pos = In.kr(positionbus, 3);

		// calculate theta, phi and rho
		cartesian = Cartesian(pos[0], pos[1], pos[2]);
		theta = cartesian.theta;
		phi = cartesian.phi;
		radialDistance = cartesian.rho;

		// calculate the attenuation of high frequencies based on radial distance
		// dB/m = freq/100000
		freq = lpfDB/radialDistance.clip(minAttenDist, maxMeters) * hiFreqAttenCoeff;

		// radialDistance = VarLag.kr(radialDistance);

		// the amplitude based on a starting amplitude, amp
		amplitude = 1;//amp / radialDistance.clip(1, maxMeters);

		// lowpass the src and apply envelope and amplitude
		chain = LPF.ar(src, freq);

		// use a delay line based on the distance to simulate a doppler shift
		chain = DelayC.ar(chain, maxMeters/speedOfSound, radialDistance/speedOfSound);

		// create the reverb
		reverb = FoaReverb.ar(chain * reverbGain.dbamp, predelay: 0.0, t60low: 0.6, t60high: 0.2, diffusion: 0.5, mod: 0.2);

		// zoom in on the center of the sound-field based on the arctangent of
		// the distance and a user-defined width
		chain = FoaZoom.ar(chain * amplitude, radialDistance.atan2(width), theta, phi);

		// use proximity filters based on the distance
		// note: proximity boots low frequencies so we need to condition our
		// signal by using a highpass
		chain = FoaProximity.ar(HPF.ar(HPF.ar(chain, 20.0), 20.0), radialDistance + 0.1);

		// tumble and then rotate
		//chain = FoaTumble.ar(chain, phi);
		//chain = FoaRotate.ar(chain, theta);

		// send out signal
		Out.ar(outbus, chain + reverb)
	});

	// Routines ---------------------------------------

	scoreRoutine = Routine({
		// Begin routine that plays throughout piece
		persistentRoutine.play;

		// Section 1: Pulse + Breath
		"Section 1 Starting".postln;
		pulseBreathRoutine.play;

		sectionOneDur.yield;
		pulseBreathRoutine.stop;
		// Section 2: Shepherd Swarm
		"Section 2 Starting".postln;
		shepherdSwarmRoutine.play;

		sectionTwoDur.yield;
		shepherdSwarmRoutine.stop;
		// Section 3: Comb Breath / Percussive Breath
		"Section 3 Starting".postln;
		breathPercRoutine.play;

		sectionThreeDur.yield;
		breathPercRoutine.stop;
	});

	persistentRoutine = Routine({
		var decoderNote, breathPositionNote, recNote, recPosNote;

		startTime = SystemClock.seconds;

		decoderNote = decoderSynth.note(target: decoderGroup)
		.inBus_(decoderBus)
		.play;

		breathPositionNote = positionSynth.note(target: positionGroup)
		.inX_(0.5)
		.inY_(0.5)
		.inZ_(0.5)
		.inXVel_(0)
		.cubeRoomSideLen_(4.57)
		.outBus_(voicePositionBus)
		.play;

		recNote = recSynth.note(target: decoderGroup)
		.inBus_(recBus)
		.buffer_(voiceRecBuf)
		.play;

		recPosNote = recPosSynth.note(target: positionGroup)
		.delay_(0.01)
		.bufDur_(recordingDur)
		.recPosOut_(recPosBus)
		.play;

		OSCdef.new(positionOSCKey, {
			arg msg;
			var handOffset = 10;
			// 0 - message name
			// 1 - has left hand? (0 or 1)
			// 2 - x position left palm
			// 3 - y position left palm
			// 4 - z position left palm
			// 5 - roll left palm (z-axis rotation)
			// 6 - pitch left palm (x-axis rotation)
			// 7 - yaw left palm (y-axis rotation)
			// 8 - x velocity of palm
			// 9 - grab strength left palm
			// 10 - switch synth? (-1, 0, 1)
			// 11 - 19, same but for the right hand

			if((msg[handOffset + 1] > 0), {
				// positioning controls -- 1 to 1 spatial mapping
				breathPositionNote.inX_(msg[handOffset + 2]);
				breathPositionNote.inY_(msg[handOffset + 3]);
				breathPositionNote.inZ_(msg[handOffset + 4]);
				breathPositionNote.inXVel_(msg[handOffset + 8]);
			});

		}, '/LeapMotion', recvPort: 5347);

	});

	pulseBreathRoutine = Routine({
		var pulseNote, breathNote;
		var breathSpatializerNote, dronePositionNote;
		var droneSpatializerNote, sidechainNote;
		var pulseBuffer, maxDelay;

		pulseBuffer = sounds[pulse];
		maxDelay = 1.0;

		breathNote = breathSynth.note(target: synthesisGroup)
		.gain_(-3)
		.dur_(sectionOneDur + sectionTwoDur)
		.delay_(1 / 10)
		.maxDelay_(maxDelay)
		.t60_(0)
		.tOct_(100)
		.pitchUp_(1.0)
		.buffer_(voiceRecBuf)
		.outBus_(voiceSpatializerBus)
		.recBus_(recBus)
		.pos_(recPosBus)
		.play;

		pulseNote = pulseSynth.note(target: synthesisGroup)
		.gain_(-3)
		.dur_(sectionOneDur + sectionTwoDur)
		.rate_(CtkControl.env(
				Env(
					[1, 1, 2] * pulseRate,
					[sectionOneDur, sectionTwoDur],
					'exp')))
		.pulseBuffer_(pulseBuffer)
		.outBus_(decoderBus)
		.sidechainOutBus_(sidechainPulseBus)
		.play;

		breathSpatializerNote = spatializerSynth.note(target: spatializerGroup)
		.inbus_(voiceSpatializerBus)
		.positionbus_(voicePositionBus)
		.outbus_(sidechainBus)
		.width_(1)
		.gain_(0)
		.maxMeters_(1000)
		.reverbGain_(-12)
		.play;

		// TODO: Cleanup dead code in sidechain
		sidechainNote = sidechainSynth.note(target: sidechainGroup)
		.gain_(CtkControl.env(
				Env(
					[-27, -27, -60],
					[sectionOneDur + sectionTwoDur, 0.05],
					5)))
		.inBus_(sidechainBus)
		.pulseBus_(sidechainPulseBus)
		.sidechainGain_(7)
		.depth_(1)
		.dur_(pulseBuffer.duration)
		.rate_(pulseRate)
		.outBus_(decoderBus)
		.play;

		OSCdef.new(oscDefKey, {
			arg msg;
			var handOffset = 10;
			// 0 - message name
			// 1 - has left hand? (0 or 1)
			// 2 - x position left palm
			// 3 - y position left palm
			// 4 - z position left palm
			// 5 - roll left palm (z-axis rotation)
			// 6 - pitch left palm (x-axis rotation)
			// 7 - yaw left palm (y-axis rotation)
			// 8 - x velocity of palm
			// 9 - grab strength left palm
			// 10 - switch synth? (-1, 0, 1)
			// 11 - 19, same but for the right hand

			if((msg[handOffset + 1] > 0), {
				// breath processing parameters
				// Y -> low t60 to high
				breathNote.t60_(msg[handOffset + 3].linexp(0, 0.3, 0.001, 20));
				// Z -> low delay time to high
				breathNote.delay_(msg[handOffset + 4].linexp(0, 1, 1 / 20, maxDelay));
				// roll -> more rotation, faster tOct.
				breathNote.tOct_(((msg[handOffset + 5].abs / pi - 0.5).abs / 0.5).linexp(0, 1, 50, 0.5));
				// clockwise roll pitch up, ccw roll pitch down
				breathNote.pitchUp_(msg[handOffset + 5].abs / pi > 0.5);
			});

		}, '/LeapMotion', recvPort: 5347);

		OSCdef.new(jerkOSCKey, {
			arg msg;

			var gFreq, timeElapsed, timeTilSectionTwo, timeLeftInSectionTwo, endTime;

			gFreq = exprand(5, 15);
			timeElapsed = (SystemClock.seconds - startTime);
			timeTilSectionTwo = max(0, sectionOneDur - timeElapsed);
			endTime = sectionOneDur + sectionTwoDur;
			timeLeftInSectionTwo = min(sectionTwoDur, endTime - timeElapsed);

			droneSynth.note(target: synthesisGroup)
			.gain_(CtkControl.env(
				Env(
					[-20, -20, -60],
					[timeTilSectionTwo, timeLeftInSectionTwo],
					5)))
			.dur_(endTime - timeElapsed)
			.gFreq_(CtkControl.env(
				Env(
					[1, 1, 4] * gFreq,
					[timeTilSectionTwo, timeLeftInSectionTwo],
					'exp')))
			.rate_(CtkControl.env(
				Env(
					[1.0, 1.0, 4],
					[timeTilSectionTwo, timeLeftInSectionTwo],
					'exp')))
			.driftRate_(0.5)
			.index_(recPosBus.getSynchronous(0))
			.buffer_(voiceRecBuf)
			.outBus_(sidechainBus)
			.play;
		}, '/tr');
	});

	shepherdSwarmRoutine = Routine({

		var host, triggerName;

		host = NetAddr.new("localhost", NetAddr.langPort);
		triggerName = '/simTr';

		// should overwrite previous OSCDef
		// todo: should have different amplitude envelope to make it shepherd like
		//       gain increase, then decrease
		// todo: kill make synths go to 0 amp through sidechain synth. timing hard.
		// todo: speed up rate is too fast
		// todo: volume sounds weird, maybe modulate the sidechain volume so that it
		//       increases during the shepherd section and then rapidly drops.
		OSCdef.new(jerkOSCKey, {
			arg msg;

			var droneDur, gFreq, timeElapsed, timeTilSectionTwo, timeLeftInSectionTwo, endTime;

			droneDur = sectionTwoDur / 2;
			gFreq = exprand(5, 15);
			timeElapsed = (SystemClock.seconds - startTime);
			timeTilSectionTwo = max(0, sectionOneDur - timeElapsed);
			endTime = sectionOneDur + sectionTwoDur;
			timeLeftInSectionTwo = min(sectionTwoDur, endTime - timeElapsed);

			droneSynth.note(target: synthesisGroup)
			.gain_(CtkControl.env(
				Env(
					[-60, -20, -60],
					[1, 1] * droneDur / 2,
					[-5, 5])))
			.dur_(endTime - timeElapsed)
			.gFreq_(CtkControl.env(
				Env(
					[1, 2.0] * gFreq,
					droneDur,
					'exp')))
			.rate_(CtkControl.env(
				Env(
					[1.0, 2.0],
					droneDur,
					'exp')))
			.driftRate_(0.5)
			.index_(recPosBus.getSynchronous(0))
			.buffer_(voiceRecBuf)
			.outBus_(sidechainBus)
			.play;
		}, triggerName);

		loop({
			var timeToWait = rand(5.0);
			timeToWait.yield;
			host.sendMsg(triggerName, "");
		});
	});

	breathPercRoutine = Routine({
		var breathPercNote;

		breathPercNote = breathPercSynth.note.play;

	});

	s.sync;

	// Routine Durations & Play ----------------------
	sectionOneDur = 60 * (60 / sounds[pulse].duration).round * sounds[pulse].duration;
	sectionTwoDur = sectionOneDur;
	sectionThreeDur = 60;


	scoreRoutine.play;

	// Draw progress bar GUI -------------------------

	window = Window.new("Progress", Rect(0, 0, Window.screenBounds.width, 50));
	userView = UserView.new(window, Rect(0, 0, window.bounds.width, window.bounds.height));
	userView.resize = 5;
	userView.drawFunc_({
		var totalTime, progress, sectionTwoNotch, sectionThreeNotch;

		totalTime = sectionOneDur + sectionTwoDur + sectionThreeDur;
		progress = (SystemClock.seconds - startTime) / totalTime;
		sectionTwoNotch = sectionOneDur / totalTime;
		sectionThreeNotch = sectionTwoDur / totalTime + sectionTwoNotch;

		Pen.fillColor_(Color.black);
		Pen.fillRect(Rect(0, 0, window.bounds.width, window.bounds.height));
		Pen.fillColor_(Color.green);
		Pen.fillRect(Rect(0, 0, window.bounds.width * progress, window.bounds.height));
		Pen.fillColor_(Color.red);
		Pen.fillRect(Rect(window.bounds.width * sectionTwoNotch, 0, 5, window.bounds.height));
		Pen.fillRect(Rect(window.bounds.width * sectionThreeNotch, 0, 5, window.bounds.height));

	});
	userView.animate = true;
	userView.frameRate = 5;
	window.front;

	CmdPeriod.doOnce({
		sounds.do({arg buffer;
			buffer.free;
		});
		voiceRecBuf.free;
		window.close;
	});
})
)

(
n = NetAddr.new("localhost", NetAddr.langPort);
n.sendMsg('/tr', "");
)
