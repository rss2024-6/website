// Function to open modal with video
function openModal(videoId) {
  const modal = document.getElementById("video-modal");
  const video = document.getElementById(videoId);
  modal.style.display = "block";
  const sourceElement = document.querySelector("#modal-video");
  console.log("source", sourceElement);
  // Set the src attribute
  sourceElement.setAttribute("src", `${videoId}`);
  // video.play();
}

// Function to close modal
function closeModal() {
  const modal = document.getElementById("video-modal");
  const video = document.getElementById("video");
  modal.style.display = "none";
  video.pause();
}

// Close modal when clicking outside of it
window.onclick = function (event) {
  const modal = document.getElementById("video-modal");
  if (event.target == modal) {
    closeModal();
  }
};

// Function to handle opening the modal based on URL parameters
function handleOpenModalFromURL() {
  const urlParams = new URLSearchParams(window.location.search);
  const videoId = urlParams.get("videoId");

  if (videoId) {
    openModal(videoId);
  }
}

// Call the function to open modal from URL parameters when the page loads
document.addEventListener("DOMContentLoaded", function () {
  handleOpenModalFromURL();
});
