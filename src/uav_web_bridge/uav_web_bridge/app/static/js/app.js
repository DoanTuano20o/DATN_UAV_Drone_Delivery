(() => {
  const q = (selector, root = document) => root.querySelector(selector);
  const qa = (selector, root = document) => Array.from(root.querySelectorAll(selector));
  const reducedMotion = window.matchMedia?.("(prefers-reduced-motion: reduce)")?.matches || false;
  const videoLogOnce = new WeakSet();
  const chromeBlockedVideos = new Set();
  const visibleLandingVideos = new WeakSet();
  let landingVideoUnlockBound = false;

  function isLandingPage() {
    return document.body?.dataset.page === "landing" || document.body?.classList.contains("landing-page");
  }

  function fmtNum(value, digits = 2) {
    if (value === null || value === undefined || Number.isNaN(Number(value))) return "N/A";
    return Number(value).toFixed(digits);
  }

  function setText(el, text) {
    if (!el) return;
    const next = String(text);
    if (el.textContent !== next) el.textContent = next;
  }

  function initSmoothAnchors() {
    qa('a[href^="#"]').forEach((link) => {
      link.addEventListener("click", (event) => {
        const target = q(link.getAttribute("href"));
        if (!target) return;
        event.preventDefault();
        target.scrollIntoView({ behavior: "smooth", block: "start" });
      });
    });
  }

  function initMobileNav() {
    const nav = q("[data-nav]");
    const toggle = q("[data-nav-toggle]");
    const menu = q("[data-nav-menu]");
    if (!nav || !toggle || !menu) return;

    const setOpen = (open) => {
      toggle.setAttribute("aria-expanded", String(open));
      menu.classList.toggle("is-open", open);
    };

    toggle.addEventListener("click", () => {
      setOpen(toggle.getAttribute("aria-expanded") !== "true");
    });

    qa("a", menu).forEach((link) => {
      link.addEventListener("click", () => setOpen(false));
    });

    document.addEventListener("click", (event) => {
      if (!nav.contains(event.target)) setOpen(false);
    });

    document.addEventListener("keydown", (event) => {
      if (event.key === "Escape") setOpen(false);
    });
  }

  function initNavTransition() {
    const nav = q("[data-nav]");
    if (!nav) return;

    const update = () => {
      nav.classList.toggle("nav-scrolled", window.scrollY > 16);
    };

    update();
    window.addEventListener("scroll", update, { passive: true });
  }

  function initLandingSubnav() {
    if (!isLandingPage()) return;
    const subnav = q("[data-landing-subnav]");
    if (!subnav) return;

    const links = qa('a[href^="#"]', subnav);
    const targets = links
      .map((link) => {
        const target = q(link.getAttribute("href"));
        return target ? { link, target } : null;
      })
      .filter(Boolean);

    if (!targets.length) return;

    const setActive = (activeLink) => {
      links.forEach((link) => link.classList.toggle("is-active", link === activeLink));
    };

    links.forEach((link) => {
      link.addEventListener("click", () => setActive(link));
    });

    if (!("IntersectionObserver" in window)) {
      setActive(targets[0].link);
      return;
    }

    const observer = new IntersectionObserver(
      (entries) => {
        const visible = entries
          .filter((entry) => entry.isIntersecting)
          .sort((a, b) => b.intersectionRatio - a.intersectionRatio)[0];
        if (!visible) return;
        const item = targets.find((target) => target.target === visible.target);
        if (item) setActive(item.link);
      },
      { rootMargin: "-32% 0px -52% 0px", threshold: [0, 0.16, 0.32, 0.5] }
    );

    targets.forEach(({ target }) => observer.observe(target));
    setActive(targets[0].link);
  }

  function initScrollReveal() {
    const items = qa("[data-animate]");
    if (!items.length) return;

    if (!("IntersectionObserver" in window)) {
      items.forEach((item) => item.classList.add("is-visible"));
      return;
    }

    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            entry.target.classList.add("is-visible");
            observer.unobserve(entry.target);
          }
        });
      },
      { threshold: 0.14 }
    );

    items.forEach((item) => {
      const rect = item.getBoundingClientRect();
      if (rect.top < window.innerHeight * 0.95 && rect.bottom > 0) {
        item.classList.add("is-visible");
      } else {
        observer.observe(item);
      }
    });
  }

  function initRipple() {
    if (reducedMotion) return;

    qa("[data-ripple], .btn, .icon-btn, .filter-btn").forEach((el) => {
      el.addEventListener("click", (event) => {
        const rect = el.getBoundingClientRect();
        const ripple = document.createElement("span");
        ripple.className = "ripple";
        ripple.style.left = `${event.clientX - rect.left}px`;
        ripple.style.top = `${event.clientY - rect.top}px`;
        el.appendChild(ripple);
        window.setTimeout(() => ripple.remove(), 680);
      });
    });
  }

  function initHeroParallax() {
    if (!isLandingPage()) return;
    const hero = q("[data-parallax]");
    if (!hero || reducedMotion) return;
    if (!window.matchMedia?.("(pointer: fine)")?.matches || window.innerWidth < 900) return;

    let nextX = 0;
    let nextY = 0;
    let frame = 0;

    const commit = () => {
      frame = 0;
      hero.style.setProperty("--mx", `${nextX.toFixed(2)}px`);
      hero.style.setProperty("--my", `${nextY.toFixed(2)}px`);
    };

    const schedule = () => {
      if (!frame) frame = requestAnimationFrame(commit);
    };

    hero.addEventListener(
      "mousemove",
      (event) => {
        if (document.hidden) return;
        const rect = hero.getBoundingClientRect();
        const mx = (event.clientX - rect.left) / rect.width - 0.5;
        const my = (event.clientY - rect.top) / rect.height - 0.5;
        nextX = mx * 18;
        nextY = my * 12;
        schedule();
      },
      { passive: true }
    );

    hero.addEventListener(
      "mouseleave",
      () => {
        nextX = 0;
        nextY = 0;
        schedule();
      },
      { passive: true }
    );
  }

  function getVideoSource(video) {
    if (!video) return false;
    const source = q("source", video);
    return video.currentSrc || video.getAttribute("src") || source?.src || source?.getAttribute("src") || source?.dataset.src || "";
  }

  function hasVideoSource(video) {
    return Boolean(getVideoSource(video));
  }

  function canBrowserPlayMp4() {
    const probe = document.createElement("video");
    if (!probe?.canPlayType) return false;
    const basic = probe.canPlayType("video/mp4");
    return basic === "probably" || basic === "maybe";
  }

  function syncPosterBackground(block, video) {
    const poster = video?.getAttribute("poster") || block?.dataset.imageSrc || "";
    if (block && poster) block.style.setProperty("--poster-url", `url("${poster}")`);
  }

  function normalizeLandingVideo(video) {
    if (!video) return;
    video.muted = true;
    video.defaultMuted = true;
    video.loop = true;
    video.autoplay = true;
    video.playsInline = true;
    video.setAttribute("muted", "");
    video.setAttribute("loop", "");
    video.setAttribute("autoplay", "");
    video.setAttribute("playsinline", "");
    video.setAttribute("webkit-playsinline", "");
  }

  function pauseVideo(video) {
    if (!video) return;
    video.closest("[data-media]")?.classList.remove("is-video-playing");
    if (!video.paused) video.pause();
  }

  function logVideoOnce(video, message, detail) {
    if (!video || videoLogOnce.has(video)) return;
    videoLogOnce.add(video);
    console.info(`[landing] ${message}`, detail || "");
  }

  function markVideoUnavailable(video, block, className, reason) {
    if (!block) return;
    block.classList.add(className, "has-fallback");
    block.classList.remove("is-video-playing", "video-fallback-active");
    if (className === "video-failed") {
      document.body?.classList.add("landing-video-failed");
    } else {
      document.body?.classList.add("no-video");
    }
    pauseVideo(video);
    if (reason) logVideoOnce(video, `${reason}. Using poster fallback.`);
  }

  function markAutoplayFallback(video, block, reason) {
    if (!block) return;
    block.classList.add("video-fallback-active", "has-fallback");
    block.classList.remove("is-video-playing");
    document.body?.classList.add("landing-video-fallback-active");
    chromeBlockedVideos.add(video);
    pauseVideo(video);
    logVideoOnce(video, reason || "Video autoplay was delayed; retrying after interaction.");
  }

  function clearAutoplayFallback(video, block) {
    chromeBlockedVideos.delete(video);
    block?.classList.remove("video-fallback-active");
  }

  function isHeroMedia(block) {
    const video = block ? q("video[data-bg-video]", block) : null;
    return video?.dataset.videoRole === "hero" || block?.dataset.videoPriority === "hero" || block?.classList.contains("hero-media-layer");
  }

  function isHeroVideo(video) {
    return video?.dataset.videoRole === "hero" || isHeroMedia(video?.closest("[data-media]"));
  }

  function shouldPlayVideo(video, force = false) {
    const block = video?.closest("[data-media]");
    if (!video || !block || !hasVideoSource(video) || document.hidden) return false;
    if (block.classList.contains("video-failed") || block.classList.contains("no-video")) return false;
    if (force || isHeroVideo(video)) return true;
    return block.dataset.visible === "true";
  }

  function scheduleVideoRetry(video, reason) {
    const block = video?.closest("[data-media]");
    if (!video || !block) return;
    const retries = Number(video.dataset.playRetries || 0);
    if (retries < 3) {
      video.dataset.playRetries = String(retries + 1);
      const delay = [300, 900, 1800][retries] || 1800;
      window.setTimeout(() => safePlay(video), delay);
      return;
    }
    block.classList.add("video-paused-fallback");
    markAutoplayFallback(video, block, "Video autoplay was delayed; retrying after interaction.");
  }

  function safePlay(video, options = {}) {
    const block = video?.closest("[data-media]");
    const force = Boolean(options.force);
    if (!shouldPlayVideo(video, force)) return;

    normalizeLandingVideo(video);
    block.classList.add("has-video");
    block.classList.remove("video-fallback-active", "video-paused-fallback");

    if (video.readyState < 2 && video.networkState === HTMLMediaElement.NETWORK_EMPTY) {
      video.load();
    }

    const pending = video.play();
    if (pending?.then) {
      pending
        .then(() => {
          video.dataset.playRetries = "0";
          clearAutoplayFallback(video, block);
          block.classList.remove("video-paused-fallback");
          block.classList.add("has-video", "video-ready", "is-video-playing");
        })
        .catch((error) => {
          if (document.hidden || (!force && block.dataset.visible === "false") || error?.name === "AbortError") return;
          scheduleVideoRetry(video, "Video autoplay was blocked or failed");
        });
    } else {
      block.classList.add("has-video", "is-video-playing");
    }
  }

  function prepareLandingVideo(block, canUseVideo) {
    const video = q("video[data-bg-video]", block);
    block.classList.add("has-fallback");

    if (!video) {
      markVideoUnavailable(null, block, "no-video");
      return null;
    }

    syncPosterBackground(block, video);
    normalizeLandingVideo(video);
    video.preload = isHeroMedia(block) ? "auto" : "metadata";

    const source = q("source", video);
    if (source?.dataset.src && !source.getAttribute("src")) {
      source.src = source.dataset.src;
      video.load();
    }

    if (!hasVideoSource(video)) {
      markVideoUnavailable(video, block, "no-video", "Video source is missing");
      return null;
    }

    video.addEventListener(
      "error",
      () => markVideoUnavailable(video, block, "video-failed", "Video decode or network error"),
      { once: true }
    );
    source?.addEventListener(
      "error",
      () => markVideoUnavailable(video, block, "video-failed", "Video source failed to load"),
      { once: true }
    );
    video.addEventListener("loadeddata", () => block.classList.add("has-video", "video-ready"), { once: true });
    video.addEventListener("canplay", () => block.classList.add("has-video", "video-ready"), { once: true });
    video.addEventListener("playing", () => {
      video.dataset.playRetries = "0";
      clearAutoplayFallback(video, block);
      block.classList.remove("video-paused-fallback");
      block.classList.add("has-video", "video-ready", "is-video-playing");
    });
    video.addEventListener("pause", () => block.classList.remove("is-video-playing"));
    video.addEventListener("ended", () => {
      if (document.hidden) return;
      video.currentTime = 0;
      if (isHeroVideo(video) || block.dataset.visible === "true") {
        safePlay(video, { force: true });
      }
    });

    if (!canUseVideo) {
      video.autoplay = false;
      pauseVideo(video);
      markVideoUnavailable(video, block, "no-video");
      return null;
    }

    block.classList.add("has-video");
    if (isHeroMedia(block)) {
      block.dataset.visible = "true";
      visibleLandingVideos.add(video);
    } else {
      block.dataset.visible = "false";
      pauseVideo(video);
    }

    return video;
  }

  function isBlockNearViewport(block) {
    const rect = block.getBoundingClientRect();
    return rect.top < window.innerHeight + 160 && rect.bottom > -160;
  }

  function resumeVisibleLandingVideos(videos) {
    videos.forEach((video) => {
      const block = video.closest("[data-media]");
      if (isHeroVideo(video) || block?.dataset.visible === "true" || visibleLandingVideos.has(video)) safePlay(video);
      else pauseVideo(video);
    });
  }

  function bindLandingVideoUnlock(videos) {
    if (landingVideoUnlockBound || !videos.length) return;
    landingVideoUnlockBound = true;

    const events = ["pointerdown", "click", "touchstart", "keydown"];
    const unlock = () => {
      chromeBlockedVideos.forEach((video) => safePlay(video, { force: true }));
      videos.forEach((video) => {
        const block = video.closest("[data-media]");
        if (isHeroVideo(video) || block?.dataset.visible === "true" || visibleLandingVideos.has(video)) {
          safePlay(video, { force: true });
        }
      });
      events.forEach((eventName) => document.removeEventListener(eventName, unlock));
    };

    events.forEach((eventName) => {
      document.addEventListener(eventName, unlock, { passive: true });
    });
  }

  function initLandingMedia() {
    if (!isLandingPage()) return;

    const bgVideos = qa("video[data-bg-video]");
    const mediaBlocks = bgVideos
      .map((video) => video.closest("[data-media]"))
      .filter((block, index, blocks) => block && blocks.indexOf(block) === index);
    if (!mediaBlocks.length) return;

    const canPlayMp4 = canBrowserPlayMp4();
    if (reducedMotion) {
      document.body.classList.add("reduced-motion");
    }

    if (!canPlayMp4) {
      document.body.classList.add("no-video");
      console.info("[landing] Browser reports no MP4 support; using poster fallback.");
    }

    const managedVideos = mediaBlocks
      .map((block) => prepareLandingVideo(block, canPlayMp4))
      .filter(Boolean);

    if (!managedVideos.length || !canPlayMp4) return;

    const heroVideos = managedVideos.filter(isHeroVideo);
    const sectionVideos = managedVideos.filter((video) => !isHeroVideo(video));

    bindLandingVideoUnlock(managedVideos);
    heroVideos.forEach((video) => safePlay(video));
    window.addEventListener(
      "load",
      () => {
        heroVideos.forEach((video) => safePlay(video, { force: true }));
        window.setTimeout(() => heroVideos.forEach((video) => safePlay(video, { force: true })), 500);
      },
      { once: true }
    );
    window.setTimeout(() => heroVideos.forEach((video) => safePlay(video, { force: true })), 500);

    if ("IntersectionObserver" in window) {
      const observer = new IntersectionObserver(
        (entries) => {
          entries.forEach((entry) => {
            const video = entry.target.matches?.("video[data-bg-video]") ? entry.target : q("video[data-bg-video]", entry.target);
            if (!video) return;
            const block = video.closest("[data-media]");
            const visible = entry.isIntersecting && entry.intersectionRatio >= 0.2;
            if (block) block.dataset.visible = visible ? "true" : "false";
            if (visible) {
              visibleLandingVideos.add(video);
              safePlay(video);
            } else if (!isHeroVideo(video)) {
              visibleLandingVideos.delete(video);
              pauseVideo(video);
            }
          });
        },
        { rootMargin: "120px 0px", threshold: [0, 0.2, 0.5] }
      );

      sectionVideos.forEach((video) => observer.observe(video));
    } else {
      sectionVideos.forEach((video) => {
        const block = video.closest("[data-media]");
        if (!block) return;
        block.dataset.visible = isBlockNearViewport(block) ? "true" : "false";
        if (block.dataset.visible === "true") {
          visibleLandingVideos.add(video);
          safePlay(video);
        }
      });
    }

    document.addEventListener("visibilitychange", () => {
      if (document.hidden) {
        managedVideos.forEach(pauseVideo);
        return;
      }
      resumeVisibleLandingVideos(managedVideos);
    });
  }

  function initClock() {
    const clocks = qa("[data-clock]");
    if (!clocks.length) return;

    const update = () => {
      const time = new Date().toLocaleTimeString();
      clocks.forEach((clock) => setText(clock, time));
    };

    update();
    window.setInterval(update, 1000);
  }

  function initAnimatedCounters() {
    const counters = qa("[data-count-to]");
    if (!counters.length) return;

    const animate = (el) => {
      const target = Number(el.dataset.countTo || 0);
      const duration = Number(el.dataset.countDuration || 900);
      const start = performance.now();

      const frame = (now) => {
        const progress = Math.min(1, (now - start) / duration);
        const eased = 1 - Math.pow(1 - progress, 3);
        setText(el, Math.round(target * eased));
        if (progress < 1) requestAnimationFrame(frame);
      };

      requestAnimationFrame(frame);
    };

    if (!("IntersectionObserver" in window)) {
      counters.forEach(animate);
      return;
    }

    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            animate(entry.target);
            observer.unobserve(entry.target);
          }
        });
      },
      { threshold: 0.35 }
    );

    counters.forEach((counter) => observer.observe(counter));
  }

  async function fetchJson(url) {
    const res = await fetch(url, { cache: "no-store" });
    if (!res.ok) throw new Error(`${url} returned ${res.status}`);
    return res.json();
  }

  function initVisibilityMotionState() {
    const update = () => {
      document.body.classList.toggle("is-hidden-tab", document.hidden);
    };

    update();
    document.addEventListener("visibilitychange", update);
  }

  window.UAVUI = {
    fmtNum,
    setText,
    fetchJson,
  };

  document.addEventListener("DOMContentLoaded", () => {
    initSmoothAnchors();
    initMobileNav();
    initNavTransition();
    initLandingSubnav();
    initScrollReveal();
    initRipple();
    initHeroParallax();
    initLandingMedia();
    initClock();
    initAnimatedCounters();
    initVisibilityMotionState();
  });
})();
