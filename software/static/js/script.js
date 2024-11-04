// Xử lý việc xem ảnh từ lịch sử
const historyImages = document.querySelectorAll('.history-image');
const currentImage = document.getElementById('current-image');
const imageTimestamp = document.getElementById('image-timestamp');
const detailTimestamp = document.getElementById('detail-timestamp');

historyImages.forEach(image => {
  image.addEventListener('click', function() {
    currentImage.src = this.src;
    imageTimestamp.textContent = `Thời gian chụp: ${this.getAttribute('data-timestamp')}`;
    detailTimestamp.textContent = this.getAttribute('data-timestamp');
  });
});

// Hiển thị thông tin chi tiết của ảnh
const viewDetailsButton = document.getElementById('view-details');
const detailsDiv = document.getElementById('details');

viewDetailsButton.addEventListener('click', function() {
  detailsDiv.style.display = detailsDiv.style.display === 'none' ? 'block' : 'none';
});
