// Script incluído no <head> de TODA página de conteúdo. Não precisa editar.
//
// 1) Se a página foi aberta sozinha (ex.: por um atalho ou pelo simulador),
//    redireciona para index.html?p=<esta página>, que mostra a página
//    dentro da moldura com Conteúdo e Pesquisar.
// 2) Dentro da moldura, avisa a moldura qual página carregou (título e
//    texto, usados na árvore e na pesquisa) e atende pedidos de
//    imprimir e de destacar termos pesquisados.
//
// A comunicação usa postMessage porque o Chrome isola arquivos file://
// entre si e não deixa uma página acessar a outra diretamente.

(function () {
  'use strict';

  // Raiz da ajuda = pasta acima de _sys/, descoberta pelo endereço deste script.
  // Assim a página funciona tanto em Help\ quanto em subpastas (Help\ECU\).
  var scripts = document.getElementsByTagName('script');
  var meuSrc = (document.currentScript || scripts[scripts.length - 1]).src;
  var raiz = meuSrc.replace(/_sys\/pagina\.js.*$/, '');
  var aqui = location.href.split('#')[0].split('?')[0];
  var arquivo = decodeURIComponent(aqui.indexOf(raiz) === 0 ? aqui.substring(raiz.length) : aqui.split('/').pop());

  var dentroDaMoldura;
  try { dentroDaMoldura = window.top !== window.self; } catch (e) { dentroDaMoldura = true; }

  if (!dentroDaMoldura) {
    location.replace(raiz + 'index.html?p=' + encodeURIComponent(arquivo + location.hash));
    return;
  }

  // ---------- utilidades ----------
  function normChar(c) {
    var n = c;
    if (n.normalize) { n = n.normalize('NFD').replace(/[\u0300-\u036f]/g, ''); }
    if (n.length !== 1) { n = c; }
    return n.toLowerCase();
  }
  // Remove acentos e passa para minúsculas mantendo o mesmo comprimento,
  // para que as posições encontradas valham também no texto original.
  function norm(s) {
    var r = '';
    for (var i = 0; i < s.length; i++) { r += normChar(s.charAt(i)); }
    return r;
  }

  function enviar(msg) {
    msg.helpMsg = 1;
    try { window.parent.postMessage(JSON.stringify(msg), '*'); } catch (e) { }
  }

  function tituloDaPagina() {
    var h1 = document.getElementsByTagName('h1')[0];
    var t = h1 ? (h1.innerText || h1.textContent) : document.title;
    return (t || arquivo).replace(/\s+/g, ' ').replace(/^\s+|\s+$/g, '');
  }

  // Texto da página com espaço entre células, itens e parágrafos
  // (innerText não serve: no iframe oculto da pesquisa ele vem sem separadores).
  var BLOCOS = /^(P|DIV|LI|TD|TH|TR|TABLE|UL|OL|H1|H2|H3|H4|H5|H6|PRE|BR|DT|DD|BLOCKQUOTE)$/;
  function textoDaPagina() {
    var partes = [];
    (function andar(no) {
      for (var f = no.firstChild; f; f = f.nextSibling) {
        if (f.nodeType === 3) { partes.push(f.nodeValue); }
        else if (f.nodeType === 1 && !/^(SCRIPT|STYLE)$/.test(f.nodeName)) {
          var bloco = BLOCOS.test(f.nodeName);
          if (bloco) { partes.push(' '); }
          andar(f);
          if (bloco) { partes.push(' '); }
        }
      }
    })(document.querySelector('.conteudo') || document.body);
    return partes.join('').replace(/\s+/g, ' ').replace(/^\s+|\s+$/g, '');
  }

  // ---------- destaque dos termos pesquisados ----------
  function limparDestaque() {
    var marcas = document.querySelectorAll('mark.hl');
    for (var i = 0; i < marcas.length; i++) {
      var m = marcas[i], pai = m.parentNode;
      pai.replaceChild(document.createTextNode(m.textContent), m);
      pai.normalize();
    }
  }

  function destacar(termos) {
    limparDestaque();
    if (!termos || !termos.length) { return; }
    var nos = [];
    (function coletar(no) {
      for (var f = no.firstChild; f; f = f.nextSibling) {
        if (f.nodeType === 3) { nos.push(f); }
        else if (f.nodeType === 1 && !/^(SCRIPT|STYLE|MARK)$/.test(f.nodeName)) { coletar(f); }
      }
    })(document.querySelector('.conteudo') || document.body);

    var primeira = null;
    for (var i = 0; i < nos.length; i++) {
      var no = nos[i], txt = no.nodeValue, tn = norm(txt), faixas = [];
      for (var t = 0; t < termos.length; t++) {
        var p = tn.indexOf(termos[t]);
        while (p >= 0) { faixas.push([p, p + termos[t].length]); p = tn.indexOf(termos[t], p + 1); }
      }
      if (!faixas.length) { continue; }
      faixas.sort(function (a, b) { return a[0] - b[0]; });
      var unidas = [faixas[0]];
      for (var k = 1; k < faixas.length; k++) {
        var u = unidas[unidas.length - 1];
        if (faixas[k][0] <= u[1]) { u[1] = Math.max(u[1], faixas[k][1]); } else { unidas.push(faixas[k]); }
      }
      var frag = document.createDocumentFragment(), pos = 0;
      for (var j = 0; j < unidas.length; j++) {
        frag.appendChild(document.createTextNode(txt.substring(pos, unidas[j][0])));
        var mk = document.createElement('mark');
        mk.className = 'hl';
        mk.appendChild(document.createTextNode(txt.substring(unidas[j][0], unidas[j][1])));
        frag.appendChild(mk);
        if (!primeira) { primeira = mk; }
        pos = unidas[j][1];
      }
      frag.appendChild(document.createTextNode(txt.substring(pos)));
      no.parentNode.replaceChild(frag, no);
    }
    if (primeira && primeira.scrollIntoView) { primeira.scrollIntoView(); window.scrollBy(0, -60); }
  }

  // ---------- mensagens vindas da moldura ----------
  window.addEventListener('message', function (ev) {
    var d;
    try { d = JSON.parse(ev.data); } catch (e) { return; }
    if (!d || !d.helpMsg) { return; }
    if (d.tipo === 'imprimir') { window.print(); }
    else if (d.tipo === 'destacar') { destacar(d.termos); }
  });

  document.addEventListener('DOMContentLoaded', function () {
    enviar({ tipo: 'pagina', arquivo: arquivo, titulo: tituloDaPagina(), texto: textoDaPagina() });
  });
})();
